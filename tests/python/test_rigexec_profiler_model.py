#!/usr/bin/env python
"""Profiler shape reports actual graph dependencies without barrier levels."""
import importlib.util
from pathlib import Path
from test_rigexec_python import _setup_environment
_setup_environment()
path=Path(__file__).resolve().parents[2]/"plugin"/"rigExecUsdview"/"profilerModel.py"
spec=importlib.util.spec_from_file_location("rigexec_profiler_model",path)
model=importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
class Rig:
    def __init__(self,nodes): self.nodes=nodes
    def op_graph(self): return self.nodes

def main():
    graph=[{"step":8,"preds":[3,7],"cluster":2,"domain":"geometry"},
           {"step":3,"preds":[1],"cluster":1,"domain":"pose"},
           {"step":1,"preds":[],"cluster":0,"domain":"head"},
           {"step":7,"preds":[1],"cluster":1,"domain":"weight"}]
    shape=model.schedule_shape(None,Rig(graph),"/Rig")
    assert shape["op_count"]==4 and shape["longest_path"]==3
    assert shape["cluster_count"]==3 and shape["domains"]["pose"]==1
    assert model.schedule_shape(None,Rig([]),"/Rig")["longest_path"]==0
    for invalid in ([{"step":0,"preds":[9],"cluster":0,"domain":"pose"}],
                    [{"step":0,"preds":[0],"cluster":0,"domain":"pose"}]):
        try: model.schedule_shape(None,Rig(invalid),"/Rig")
        except ValueError: pass
        else: raise AssertionError("invalid graph accepted")
    print("RIGEXEC_PROFILER_MODEL_OK")
if __name__=="__main__": main()
