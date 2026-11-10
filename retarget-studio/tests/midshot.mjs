import puppeteer from 'puppeteer';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { extname, join } from 'node:path';
const ROOT = process.cwd();
const MIME = { '.html':'text/html','.js':'text/javascript','.mjs':'text/javascript','.css':'text/css','.json':'application/json' };
const server = createServer(async (req,res)=>{
  let p=req.url.split('?')[0]; if(p==='/')p='/index.html';
  try{ const f=join(ROOT,p); const d=await readFile(f);
    let t=MIME[extname(f)]; if(!t) t=f.includes('/examples/jsm/')?'text/javascript':'application/octet-stream';
    res.writeHead(200,{'content-type':t}); res.end(d);
  }catch{res.writeHead(404);res.end();}
});
await new Promise(r=>server.listen(0,r));
const browser=await puppeteer.launch({headless:'new',executablePath:process.env.CHROME_BIN,
  env:{...process.env,LD_LIBRARY_PATH:process.env.CHROME_LD_PATH,FONTCONFIG_FILE:process.env.FONTCONFIG_FILE},
  args:['--no-sandbox','--disable-gpu','--enable-unsafe-swiftshader','--use-gl=angle','--use-angle=swiftshader']});
const page=await browser.newPage();
await page.setViewport({width:1500,height:1000});
await page.goto(`http://127.0.0.1:${server.address().port}/index.html`,{waitUntil:'networkidle0'});
await page.waitForSelector('#check-results .check-item');
// scrub to u=0.25 (max arm swing), enable ghost overlay
await page.evaluate(()=>{ const s=document.getElementById('scrub'); s.value=250; s.dispatchEvent(new Event('input'));
  const g=document.getElementById('ghost'); g.checked=true; g.dispatchEvent(new Event('change')); });
await new Promise(r=>setTimeout(r,400));
await page.screenshot({path:'tests/midpose.png'});
// also capture validation panel scrolled into view
await page.evaluate(()=>document.getElementById('panel-validation').scrollIntoView());
await new Promise(r=>setTimeout(r,200));
await page.screenshot({path:'tests/checks.png'});
await browser.close(); server.close(); console.log('done');
