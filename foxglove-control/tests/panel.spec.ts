import {test,expect} from '@playwright/test';
import path from 'node:path';
import {enabled} from '../src/model';

test('phase gates and emergency stop',()=>{
 const active={fresh:true,services:true,busy:false};
 const s={workflow_phase:'ready',workflow_revision:'g:1',camera_live:true};
 expect(enabled('start',s,active)).toBe(true);
 expect(enabled('start',s,{...active,fresh:false})).toBe(false);
 expect(enabled('save',{...s,workflow_phase:'review',last_episode_complete:false},active)).toBe(false);
 expect(enabled('discard',{...s,workflow_phase:'review'},active)).toBe(true);
 expect(enabled('start',{...s,workflow_phase:'resetting'},active)).toBe(false);
 expect(enabled('stop',s,{...active,fresh:false,busy:true})).toBe(true);
 expect(enabled('stop',s,{...active,services:false})).toBe(false);
});

test('actual bundled panel sends exact commands, expires telemetry and shows metrics',async({page})=>{
 await page.setViewportSize({width:680,height:1000});
 await page.setContent('<body style="margin:0"><div id="panel"></div></body>');
 await page.evaluate(()=>{(window as any).module={exports:{}};});
 await page.addScriptTag({path:path.resolve('dist/extension.js')});
 await page.evaluate(()=>{
  const w=window as any;
  w.requests=[];
  w.ctx={panelElement:document.getElementById('panel'),watch:()=>{},subscribe:()=>{},unsubscribeAll:()=>{},callService:async(service:string,payload:unknown)=>{w.requests.push({service,payload});return {ok:true,message:'Accepted'};}};
  w.module.exports.activate({registerPanel:({initPanel}:any)=>{w.cleanup=initPanel(w.ctx);}});
 });
 await expect(page.getByRole('heading',{name:'Panthera Collection'})).toBeVisible();
 expect(await page.evaluate(()=>(window as any).requests.length)).toBe(0);
 await page.evaluate(()=>{
  const w=window as any;
  w.state={workflow_phase:'ready',workflow_revision:'run:3',camera_live:true,camera_latency_ms:18.2,camera_age_ms:4,camera_fps:30,teleop_state:'idle',task_catalogs:[{id:'c1',name:'Tabletop pick and place',tasks:[{id:'t1',prompt:'Place the red block in the bowl.'}]}],selected_catalog_id:'c1',selected_task:{catalog_id:'c1',task_id:'t1',prompt:'Place the red block in the bowl.'},upload_repo:'Mirage415/nyush_panthera_test',upload_state:'ready'};
  w.emit=()=>w.ctx.onRender({colorScheme:'dark',currentFrame:[{topic:'/capture/status',message:w.state}]},()=>{});
  w.emit();w.timer=setInterval(w.emit,200);
 });
 await expect(page.getByText('18.2 ms',{exact:true})).toBeVisible();
 await expect(page.getByRole('button',{name:'Start',exact:true})).toBeEnabled();
 await expect(page.getByRole('textbox',{name:'Subtask / prompt'})).toHaveValue('Place the red block in the bowl.');
 await page.getByRole('textbox',{name:'Subtask / prompt'}).fill('Pick up the blue cup.');
 await expect(page.getByRole('button',{name:'Start',exact:true})).toBeDisabled();
 await page.getByRole('button',{name:'Use prompt',exact:true}).click();
 expect(await page.evaluate(()=>(window as any).requests[0])).toEqual({service:'/capture/command',payload:{action:'task_create',revision:'run:3',catalog_id:'c1',prompt:'Pick up the blue cup.'}});
 await page.getByRole('textbox',{name:'Subtask / prompt'}).fill('Place the red block in the bowl.');
 await page.evaluate(()=>{(window as any).requests=[];});
 await page.screenshot({path:'tests/output/ready.png',fullPage:true});
 await page.getByRole('button',{name:'Start',exact:true}).click();
 expect(await page.evaluate(()=>(window as any).requests[0])).toEqual({service:'/capture/command',payload:{action:'start',revision:'run:3'}});
 await page.evaluate(()=>{const w=window as any; w.state={...w.state,workflow_phase:'recording',workflow_revision:'run:4',recording:true,teleop_state:'running',follower_live:true,follower_age_s:.012,elapsed_s:12.3};w.emit();});
 await expect(page.getByRole('button',{name:'Save',exact:true})).toBeEnabled();
 await expect(page.getByRole('button',{name:'Start',exact:true})).toBeDisabled();
 await page.getByRole('button',{name:'Discard',exact:true}).click();
 expect(await page.evaluate(()=>(window as any).requests[1].payload.action)).toBe('discard');
 await page.evaluate(()=>{const w=window as any;w.state={...w.state,workflow_phase:'reset_required',workflow_revision:'run:5',recording:false,teleop_state:'idle',reset_available:true};w.emit();});
 await expect(page.getByRole('button',{name:'Reset Follower',exact:true})).toBeEnabled();
 await page.getByRole('button',{name:'Reset Follower',exact:true}).click();
 expect(await page.evaluate(()=>(window as any).requests[2].payload.action)).toBe('reset_follower');
 await page.evaluate(()=>clearInterval((window as any).timer));
 await expect(page.getByText('Disconnected / stale',{exact:true})).toBeVisible({timeout:3000});
 await expect(page.getByRole('button',{name:'Reset Follower',exact:true})).toBeDisabled();
 await expect(page.getByRole('button',{name:'Stop',exact:true})).toBeEnabled();
 await page.evaluate(()=>(window as any).cleanup());
});
