"""Inspect PE/ELF architecture and Linux exports without loading foreign code."""
import hashlib
import json
import re
import struct
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def inspect(path,header,prefix):
    blob=path.read_bytes()
    result={'file':str(path.relative_to(ROOT)),'bytes':len(blob),'sha256':hashlib.sha256(blob).hexdigest()}
    if path.suffix=='.dll':
        assert blob[:2]==b'MZ'
        nt=struct.unpack_from('<I',blob,0x3c)[0]
        assert blob[nt:nt+4]==b'PE\0\0'
        assert struct.unpack_from('<H',blob,nt+4)[0]==0x8664
        assert struct.unpack_from('<H',blob,nt+24)[0]==0x20b
        result.update(format='PE32+ x86_64 DLL')
        return result
    assert blob[:6]==b'\x7fELF\x02\x01'
    eh=struct.unpack_from('<16sHHIQQQIHHHHHH',blob,0)
    assert eh[1:3]==(3,62),'Expected x86_64 ELF shared object'
    sections=[struct.unpack_from('<IIQQQQIIQQ',blob,eh[6]+i*eh[11]) for i in range(eh[12])]
    def strings(section):return blob[section[4]:section[4]+section[5]]
    def name(table,offset):return table[offset:].split(b'\0',1)[0].decode('utf-8')
    exports=set();needed=[]
    for section in sections:
        if section[1]==11:
            table=strings(sections[section[6]])
            for offset in range(section[4],section[4]+section[5],section[9]):
                n,info,visibility,index,_,_=struct.unpack_from('<IBBHQQ',blob,offset)
                if info>>4 in (1,2) and index and visibility&3==0:exports.add(name(table,n))
        if section[1]==6:
            table=strings(sections[section[6]])
            for offset in range(section[4],section[4]+section[5],section[9]):
                tag,value=struct.unpack_from('<qQ',blob,offset)
                if tag==1:needed.append(name(table,value))
    required=set(re.findall(r'\b('+prefix+r'\w+)\s*\(',header.read_text(encoding='utf-8')))
    assert required<=exports,sorted(required-exports)
    assert not any(n.startswith(('rtInf','rtNaN','rt_Init','PX4Controller_','AVANTI2_')) for n in exports)
    assert not any(re.search(r'(?:libmw|libmex|libmx|libmat\.)',n) for n in needed)
    versions=sorted(set(v.decode() for v in re.findall(rb'GLIBC_\d+\.\d+(?:\.\d+)?',blob)),key=lambda v:tuple(map(int,v[6:].split('.'))))
    assert not versions or tuple(map(int,versions[-1][6:].split('.'))) <= (2,35)
    result.update(format='ELF64 x86_64 shared object',needed=needed,abi_exports=sorted(required),glibc_versions=versions)
    return result

if __name__=='__main__':
    reports=[]
    for folder,stem,header,prefix in [('fixedwing','avanti2','avanti_c_api.h','avanti_'),('fixedwing_att','avanti2_att','avanti_att_c_api.h','avanti_att_')]:
        root=ROOT/'runtime'/folder
        for filename in (stem+'.dll','lib'+stem+'.so'):
            reports.append(inspect(root/'lib'/filename,root/'include'/header,prefix))
    output=ROOT/'build/test_output/runtime_artifacts.json';output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(reports,indent=2),encoding='utf-8')
    print(json.dumps(reports,indent=2))
