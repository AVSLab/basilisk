#!/bin/bash
# ============================================================================
#  stocheff production status check -- READ-ONLY, changes nothing.
#  Copy this file to the cluster (or it is already in slurm/ after a git pull)
#  and run:   bash status_check.sh
#  Then paste the whole output back.
# ============================================================================

# --- paths (edit only if your layout differs) ---
export BSKROOT=/projects/$USER/stochastic_mc/basilisk
export STUDY=$BSKROOT/examples/dynamicsComparison/stochasticEfficiency
export PY=$BSKROOT/.venv/bin/python
export RES=/scratch/alpine/$USER/stocheff/results
export PRODID=29749899          # production array job id
cd "$STUDY" || { echo "cannot cd to $STUDY"; exit 1; }

echo "########## 1. Is the numba-cache fix deployed on this checkout? ##########"
git -C "$BSKROOT" log --oneline -1
echo "XDG_CACHE_HOME lines in slurm/env.sh (1 = fix present, 0 = old env.sh): $(grep -c XDG_CACHE_HOME slurm/env.sh)"
echo

echo "########## 2. What does the PLAN say tasks should cost? ##########"
$PY -c "
import json; p=json.load(open('$RES/plan.json'))
print('total %.0f core-h, %d tasks' % (p['summary']['totalCoreHours'], p['summary']['nArrayTasks']))
for k,c in sorted(p['configs'].items(), key=lambda kv:-kv[1]['estWallCoreHours'])[:10]:
    print('  %-26s N=%9g shards=%3d est=%.1f core-h' % (k,c['nSamples'],c['nShards'],c['estWallCoreHours']))
"
echo

echo "########## 3. Job state tally (production array) ##########"
sacct -j "$PRODID" --format=State -n | sort | uniq -c
echo

echo "########## 4. Where are the production .out logs? ##########"
ls -1 stocheff-prod-*.out 2>/dev/null | head
find "$BSKROOT" -maxdepth 6 -name 'stocheff-prod-*.out' -mmin -600 2>/dev/null | head
echo

echo "########## 5. Tail a running task's log (array index 0) ##########"
L=$(ls -t stocheff-prod-${PRODID}_0.out stocheff-prod-*_0.out 2>/dev/null | head -1)
echo "log file: ${L:-<none found -- check the paths printed in step 4>}"
[ -n "$L" ] && tail -25 "$L"
echo

echo "########## 6. Are samples growing? (count + newest mtimes + largest files) ##########"
echo "npz count now: $(ls "$RES"/samples/*.npz 2>/dev/null | wc -l)"
echo "-- 8 most-recently-written sample files --"
ls -la --time-style=+%m-%d_%H:%M "$RES"/samples/*.npz 2>/dev/null | sort -k6 | tail -8
echo "-- largest sample files (is any config's data actually big?) --"
ls -laS --time-style=+%m-%d_%H:%M "$RES"/samples/*.npz 2>/dev/null | head -6
echo

echo "########## 7. Which config is each array index? (map task 0/1/58/81) ##########"
$PY -c "
import sys; sys.path.insert(0,'$STUDY')
import runComparison as rc, stochasticDragModel as m, json
p=json.load(open('$RES/plan.json'))
cfgs=rc.defaultConfigGrid()+[rc.referenceConfig()]
params=m.ScenarioParams(orbits=2.0, stationaryStd=0.3)
tasks=[]
for c in cfgs:
    spec=p['configs'].get(c.key())
    if not spec: continue
    nsh=int(spec['nShards'])
    if nsh<=1: tasks.append((c.key(),None,c.hash(params)))
    else:
        for s in range(nsh): tasks.append((c.key(),s,c.hash(params)))
for i in [0,1,58,81]:
    if i<len(tasks): print('  array idx %2d -> %-24s shard=%s hash=%s' % (i,tasks[i][0],tasks[i][1],tasks[i][2]))
print('  (total plan tasks: %d)'%len(tasks))
"
echo
echo "########## done -- paste everything above ##########"
