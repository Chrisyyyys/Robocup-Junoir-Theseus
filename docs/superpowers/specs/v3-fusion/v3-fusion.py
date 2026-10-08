"""Entry point for Fusion's Scripts and Add-Ins dialog: builds the Theseus V3 baseline (see README.md)."""
import importlib
import os
import sys
import traceback
import adsk.core

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        import fusion_lib
        import v3_model
        importlib.reload(fusion_lib)
        importlib.reload(v3_model)
        v3_model.build('all', reset=True)
        ui.messageBox('Theseus V3 baseline built. See v3-fusion/README.md for what is modelled and what is not.')
    except Exception:
        ui.messageBox(traceback.format_exc())
