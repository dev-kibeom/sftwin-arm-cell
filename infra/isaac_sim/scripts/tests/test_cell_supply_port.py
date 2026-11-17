"""Supply request acceptance belongs to the selected Cell."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from isaac_runtime import cell_supply as port


def test_requests_validate_inputs_and_preserve_owner_rejection():
    owner=object();calls=[]
    def request(material,count):
        calls.append((material,count))
        return dict(accepted=False,detail='busy')
    port.register_supply_port(owner,dict(materials=[dict(id='TapeRoll17')],count_label='cartridges'),
                              request,lambda:dict(available=True,state='infeed'))
    try:
        for count in (True,0,-1,1.2):
            assert not port.request_supply('TapeRoll17',count)['accepted']
        assert not port.request_supply('unknown',1)['accepted']
        assert calls==[]
        assert port.request_supply('TapeRoll17',2)==dict(accepted=False,detail='busy')
        assert calls==[('TapeRoll17',2)]
        descriptor=port.supply_descriptor();descriptor['materials'].clear()
        assert port.supply_descriptor()['materials']
        port.clear_supply_port(object())
        assert port.supply_status()['state']=='infeed'
    finally:port.clear_supply_port(owner)
    assert not port.request_supply('TapeRoll17',1)['accepted']
