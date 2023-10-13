var http    = require('http'),
    path    = require('path'),
    fs      = require('fs');
print=console.log

var URL = 'localhost:6666'

function POST(url, data, cb) {
  var params,headers;
  if (data && data.params && data.data != undefined) {
    params=data.params;
    headers=data.headers;
    data=data.data;
  }
  var ishttps= url.match(/https:/);
  url=url.replace(/http[s]?:\/\//,'');
  var parts = url.split(':'),
      path  = parts[0].split('/').slice(1).join('/'),
      host  = parts[0].split('/')[0],
      port  = parts[1]||(ishttps?'443':'80');
  if (params) {
        var o=params,sep='';
        params='/?';
        for(var p in o) {
          params = params + sep + p + '='+o[p];
          sep='&';
        } 
  } else params='';
  var post_data = typeof data == 'string'?data:JSON.stringify(data);
  var post_options = {
      host: host,
      port: port,
      path: '/'+path+params,
      method: 'POST',
      keepAlive: true,
      headers: headers || {
          'Content-Type': 'application/json', // ?? 'application/x-www-form-urlencoded',
          // 'Content-Length': Buffer.byteLength(post_data)
          'Content-Length': post_data.length,
      }
  };
  // console.log('POST', post_options,post_data)
  var post_req = (ishttps?https:http).request(post_options, function(res) {
      res.setEncoding('utf8');
      
      var data='';
      res.on('data', function (chunk) {
        data += chunk;
        // console.log('Response: ' + chunk);
      });
      res.on('end', function () {
        try {
          var result=JSON.parse(data);
          // console.log('POST: ',result);
        } catch (e) { print(data); result=e; }
        if (cb) cb(result);
      });
  });
  post_req.on('error',function (err) {
    if (cb) cb(err); else console.log(url,err)
  });
  post_req.setNoDelay();
  // console.log('POST: ',post_data);
  // post the data
  post_req.write(post_data);
  post_req.end();
}

function test() {
  var req = {
    command : 'sim1',
    parts : [
      {name:'plate1metal',density:5,data:fs.readFileSync('../demo3/plate1_metal.stl','utf8') },
      {name:'plate1fibre',density:2,data:fs.readFileSync('../demo3/plate1_fibre.stl','utf8') },
    ],
    arguments : {
      energy : 0.1,
      width : 1000,
      height : 1000
    }
  }
  var body = JSON.stringify(req)
  print(body.length)
  POST(URL,body, function (result) {
    if (result instanceof Error) return print(result);
    var image = result.image
    image.data=new Uint32Array(image.data);
    print(image)
  })
}

function savePGM(name, image, range) {
  var header = 'P2\n'+image.width+' '+image.height+'\n'+range+'\n',
      data = [],
      min  = image.min,
      max  = image.max,
      scale  = range/(image.max-image.min)
  // auto scale
  for (var i=0;i<image.data.length;) {
    var row = []
    for(var k=0;k<8;k++) {
      row.push(((image.data[i]-min)*scale)|0)
      i++;
      if (i==image.data.length) break;
    }
    data.push(row.join(' '))
  }
  fs.writeFileSync(name,header+data.join('\n'),"utf8");
}

function usage () {
  print("usage: xraysim [-ip URL] [-s(ourcePos) x y z] [-d(etPos) x y z] [-D(etUpVec) x y z");
  print("               [-S(scale) min max range (linear)] [-e(nergy) MeV]");
  print("               [-p pixelsize] [-w(idth) pixels] [-h(eight) pixels] ");
  print("               [-r(otate-z) degree] [-R(otate) x y z]");
  print("               [-o out.pgm|tif(f)]");
  print("               [-u density g/cm3] file.stl [-u density file2.stl ..]");
}
function sim() {
  var arguments = {},
      densities = [],
      output = 'xray.pgm',
      parts = []
  for(var i=2;i<process.argv.length;i++) {
    var arg=process.argv[i];
    switch (arg) {
      case '-help': return usage();
      case '-e': arguments.energy=process.argv[++i]; break;
      case '-r': arguments.rotate=process.argv[++i]; break;
      case '-R': arguments.Rotate = [process.argv[++i],process.argv[++i],process.argv[++i]]; break; 
      case '-w': arguments.width=process.argv[++i]; break;
      case '-h': arguments.height=process.argv[++i]; break;
      case '-u': densities.push(process.argv[++i]); break;
      case '-p': arguments.pixelSize=process.argv[++i]; break;
      case '-o': output=process.argv[++i]; break;
      case '-ip': URL=process.argv[++i]; break;
      case '-s': arguments.sourcePos   = [process.argv[++i],process.argv[++i],process.argv[++i]]; break; 
      case '-d': arguments.detectorPos = [process.argv[++i],process.argv[++i],process.argv[++i]]; break; 
      case '-D': arguments.detectorUpVec = [process.argv[++i],process.argv[++i],process.argv[++i]]; break; 
      case '-S': arguments.scale         = [process.argv[++i],process.argv[++i],process.argv[++i]]; break; 
      default:
        if (arg[0]!='-') {
          parts.push(arg);
        }
    }
  }
  if (parts.length > 1 && parts.length != densities.length) throw "Parts and densities have different lengths";
  for(var i=0;i<parts.length;i++) {
    parts[i]={
      name:path.basename(parts[i]),
      density:densities[i]||1,
      data:fs.readFileSync(parts[i],'utf8')
    }
  }
  var req = {
    command:'sim1',
    parts:parts,
    arguments:arguments
  }
  var body = JSON.stringify(req)
  print('sending request body',body.length)
  POST(URL,body, function (result) {
    if (result instanceof Error) return print(result);
    print(result.stdout)
    var image = result.image
    image.data=new Uint32Array(image.data);
    // print(image)
    savePGM(output,image,(req.arguments.scale && req.arguments.scale[2]) || 65535)
  })
}
sim()
