"""Plot recorded tracking measurements only; never imports the simulator."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence',type=Path)
    args=parser.parse_args()
    fig,axis=plt.subplots(figsize=(9,5),layout='constrained')
    for speed in (1.0,.5,.25):
        root=args.evidence/('speed_'+str(speed))
        result=json.loads((root/'result.json').read_text())
        rows=json.loads((root/'control_rows.json').read_text())[24:]
        x=[r.get('trajectory_row',i+1)/result['full_reference_rows'] for i,r in enumerate(rows)]
        y=[1000*r['max_tracking_error_rad'] for r in rows]
        axis.plot(x,y,label=f'{speed:g}x speed: {result["status"]}')
        axis.scatter(x[-1],y[-1],s=20)
    axis.axhline(20,color='firebrick',linestyle='--',label='unchanged 20 mrad tracking guard')
    axis.set(xlim=(0,1),ylim=(0,23),xlabel='Fraction of the same reference trajectory executed',
             ylabel='Maximum measured joint tracking error (mrad)',
             title='Full-scene motor commissioning of frozen accepted path\nNot grasping, folding or physical-locking evidence')
    axis.grid(alpha=.25);axis.legend(loc='upper right')
    fig.savefig(args.evidence/'tracking_error.png',dpi=160)
    print(args.evidence/'tracking_error.png')


if __name__=='__main__':main()
