"""Generate a MAP lookup: python -m control.map_generate model.json output.npz"""
import argparse
import json
from .map_lut import generate

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('model')
    parser.add_argument('output')
    parser.add_argument('--speed-min',type=float,default=.9)
    parser.add_argument('--speed-max',type=float,default=7.)
    args=parser.parse_args()
    with open(args.model) as f: model=json.load(f)
    import numpy as np
    generate(args.output,model,speeds=np.linspace(args.speed_min,args.speed_max,65))
if __name__=='__main__': main()
