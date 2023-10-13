// xraysim server

var proc    = require('child_process');
var http    = require('http')
var fs      = require('fs')
var Path    = require('path')
var execSync  = require("child_process").execSync;
var tiff = require('./UTIF')
var binary = "./xraysim"

log=console.log
if (!fs.rmSync) fs.rmSync = function (path,options) {
  log('Deleting '+path)
  execSync('rm -rf '+path);
}
// gamma scaled by xraysim?
function decodeTIFF(file) {
  var data = Buffer.from(fs.readFileSync(file,'binary'),'binary');
  // log(file,data.length,typeof data)
  var ifd = tiff.decode(data);
  tiff.decodeImage(data,ifd[0]);
  if (!ifd[0].width || !ifd[0].height) throw Error("TIFF decoding failed");
  var image = {
    width    : ifd[0].width,
    height   : ifd[0].height,
    data     : new Uint32Array(new Uint8Array(ifd[0].data).buffer),
    depth    : 1,
    type     : 'UINT32',
  }
  var min,max
  min = max = image.data[0];for(var i=1;i<image.data.length;i++) {
    min=Math.min(min,image.data[i])
    max=Math.max(max,image.data[i])
  }
  image.min=min;
  image.max=max;
  log(file,data.length,typeof data,image.width,image.height,image.data.length,min,max)
  return image
}
// already scaled by xraysim (linear scaling)
function decodePGM(file) {
  var data = fs.readFileSync(file,'utf8'),
      lines = data.split('\n');
  if (lines[0]!='P2') return null;
  console.log(lines.slice(0,4));
  var shape = lines[1].split(' '),
      width = Number(shape[0]),
      height = Number(shape[1]),
      range = Number(lines[2])||4095;
  console.log('PGM',width,height,range)
  var image = {
    width    : width,
    height   : height,
    data     : new Uint16Array(width*height),
    depth    : 1,
    type     : 'UINT16',
  }
  var offset=0,size=width*height;
  for(var i=3;i<lines.length;i++) {
    var row = lines[i].trim().split(' ').map(Number).map(x => (x));
    if (offset<size)
      image.data.set(row,offset);
    offset+=row.length;
  }
  image.min=0;
  image.max=range;
  log(file,data.length,typeof data,image.width,image.height,image.data.length,image.min,image.max)
  return image
}

// decodeTIFF('/tmp/xray.tif')

/*
** Parse query string '?attr=val&attr=val... and return parameter record
*/
function parseQueryString( url ) {
    var queryString = url.substring( url.indexOf('?') + 1 );
    if (queryString == url) return [];
    var params = {}, queries, temp, i, l;
    // Split into key/value pairs
    queries = queryString.split("&");
    // Convert the array of strings into an object
    for ( i = 0, l = queries.length; i < l; i++ ) {
        temp = queries[i].split('=');
        if (temp[1]==undefined) temp[1]='true';
        params[temp[0]] = temp[1].replace('%20',' ');
    }
    return params;
}
function reply(response,body,headers) {
  header={'Access-Control-Allow-Origin': '*',
          'Access-Control-Allow-Credentials': 'true',
          'Content-Type': 'text/plain'};
  if (headers) { header={}; for(var p in headers) header[p]=headers[p] };
  response.writeHead(200,header);
  response.write(body);
  response.end();
}

function rpc(cmd,response) {
  switch (cmd.command) {
    case 'sim1':
      // one projection
      // parts:[{name:string,material,density:number,data:stl string}]
      var temp = fs.mkdtempSync('/tmp/');
      for(var i in cmd.parts) {
        var part = cmd.parts[i];
        fs.writeFileSync(temp+'/'+part.name,part.data);
      }
      var scale,
          arguments = []
      for(var p in cmd.arguments) {
        var arg=cmd.arguments[p];
        switch (p) {
          case 'energy':    arguments.push('-e '+arg); break;
          case 'rotate':    arguments.push('-r '+arg); break;
          case 'Rotate':    arguments.push('-R '+arg.join(' ')); break;
          case 'sourcePos'     : arguments.push('-s '+arg,join(' ')); break;
          case 'detectorPos'   : arguments.push('-d '+arg,join(' ')); break;
          case 'detectorUpVec' : arguments.push('-D '+arg.join(' ')); break;
          case 'width':     arguments.push('-w '+arg); break;
          case 'height':    arguments.push('-h '+arg); break;
          case 'pixelSize': arguments.push('-p '+arg); break;
          case 'scale'    : scale=arg; arguments.push('-S '+arg.join(' ')); break;
        }
      }
      if (!scale) arguments.push('-o '+temp+'/xray.tif'); // gamma scaled
      else        arguments.push('-o '+temp+'/xray.pgm'); // linear scaled
      
      for(var i in cmd.parts) {
        var part = cmd.parts[i];
        arguments.push('-u '+part.density);
        arguments.push(temp+'/'+part.name);
      }
      var stdout = execSync(binary+' '+arguments.join(' '));
      log(stdout.toString())
      try {
        if (!scale) {
          var image  = decodeTIFF(temp+'/xray.tif');
          image.data = Array.from(image.data)
        } else {
          var image  = decodePGM(temp+'/xray.pgm',scale);
          image.data = Array.from(image.data)        
        }
      } catch (e) {
        log(e)
      }
      fs.rmSync(temp, { recursive: true, force: true });
      var body = JSON.stringify({
        image  : image,
        stdout : stdout.toString()   
      });
      log(body.length)
      if (response) reply(response,body)
      break;
  }
}

function server (config) {
  var webSrv = http.createServer(function (request,response) {
    var body,header,sep,query,res,now,path,
        rid = request.connection.remoteAddress+':'+request.connection.remotePort;
    if (request.url.length) 
      query=parseQueryString(request.url);
    else 
      query={}

    path=request.url.match(/\/([^?]+)/);
    if (path) path=path[1];
    switch (request.method) {
      case 'GET':
        break;
      case 'POST':
        body = '';
        request.on('data', function (chunk) {
          body = body + chunk;
        });
        request.on('end', function () {
          try {
            var cmd = JSON.parse(body);
            rpc(cmd,response)
          } catch (e) {
            log(e)
          }  
        })
        break;
    }
  })

  webSrv.on("connection", function (socket) {
      // socket.setNoDelay(true);
  });

  webSrv.on("error", function (err) {
    log(err)
    process.exit();
  });

  webSrv.listen(config.port,function (err) {
    log('HTTP Service started (http://localhost:'+config.port+')');
  });
}

function test() {
  var req = {
    command : 'sim1',
    parts : [
      {name:'plate1metal',density:5,data:fs.readFileSync('../demo3/plate1_metal.stl','utf8') },
      {name:'plate1fibre',density:2,data:fs.readFileSync('../demo3/plate1_fibre.stl','utf8') },
    ],
    arguments : {
      energy : 100,
    }
  }
  log(JSON.stringify(req).length)
  rpc(req)
}

server({
  port:6666
})
