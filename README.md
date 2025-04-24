# SkeletonsAV

## Team Member

**Weizhe Jiao** weizhe.jiao@vanderbilt.edu

**Jiarui Xue** jiarui.xue@vanderbilt.edu

## How to run

1. Set up the server, in terminal, move to VehicleSim file.

```
cd VehicleSim
julia +1.9.3 --project --threads=auto
using VehicleSim
server(2; measure_all = true) # set number of cars to 2, default 1 if not passing any number
```

Move to SkeletonsAVStack file (**this repo**)
Then for groundtruth mode:

```
julia +1.9.3 --project --threads=auto
using Revise, Sockets, SkeletonsAVStack
SkeletonsAVStack.my_client(ip"your_IPADRESSS"; use_gt = true) #
```

For off mode, just set use_gt = false:

```
julia +1.9.3 --project --threads=auto
using Revise, Sockets, SkeletonsAVStack
SkeletonsAVStack.my_client(ip"your_IPADRESSS"; use_gt = false)
```

or just

```
SkeletonsAVStack.my_client(ip"your_IPADRESSS")
```

## Demo Video:

**https://youtu.be/3cTSvP8s6LQ**

## Timeline

Milestone, Target Date, Description

1. Project Setup & Requirements, **Mar 21**, Finalize scope, roles, and set up repository/environment.
2. Map & Routing Implementation, **Mar 28**, Convert RoadSegment dictionary to graph form; implement & test routing.
3. Trajectory Generation Prototype, **April 1**, Implement optimization-based planner to follow lane boundaries & speed limits on a single segment.
4. Integration w/ Localization, **April 3**, Add EKF-based sensor fusion & integrate real-time updates into the planner.
5. Full-Stack Simulation, **April 7**, Combine routing, motion planning, and localization.
6. Testing & Debugging, **April 13**, Refine trajectory generation; ensure correct stops at stop signs/loading zones.
7. Final Demo & Documentation, **April 17**, Complete final demonstration, produce project documentation, and wrap up.

Ackowledgement: We used ChatGPTO1 to reformat the code.
