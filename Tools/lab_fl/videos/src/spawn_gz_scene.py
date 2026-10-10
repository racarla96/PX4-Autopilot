import json, subprocess, sys
S=sys.argv[1]
def create(sdf):
    r=subprocess.run(['gz','service','-s','/world/default/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req','sdf: "%s"'%sdf.replace('"','\\"').replace('\n',' ')],capture_output=True,text=True)
    return 'true' in r.stdout
print(create(open(S+'/cam2.sdf').read()))
ref=json.load(open(S+'/refpath.json'))
for i,(x,y,z) in enumerate(ref):
    rad = 0.18 if i in (0,len(ref)-1) else 0.07
    col = '0 0.6 0 1' if i==0 else '1 0 0 1'
    create('<sdf version="1.9"><model name="ref%d"><static>true</static><pose>%f %f %f 0 0 0</pose><link name="l"><visual name="v"><geometry><sphere><radius>%f</radius></sphere></geometry><material><ambient>%s</ambient><diffuse>%s</diffuse><emissive>%s</emissive></material></visual></link></model></sdf>'%(i,y,x,-z,rad,col,col,col))
print('dots', len(ref))
