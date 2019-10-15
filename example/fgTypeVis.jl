# Visualize 2 sessions in Arena using local DistributedFactorGraphs types
using UUIDs
using IncrementalInference
using RoME
using DistributedFactorGraphs
using Amphitheatre


##
# create a local fg hexslam
# start with an empty factor graph object

fg1 = GraphsDFG{SolverParams}(params=SolverParams())
# Add the first pose :x0
v = addVariable!(fg1, :x0, Pose2, labels=[:POSE])
# Add at a fixed location PriorPose2 to pin :x0 to a starting location (10,10, pi/4)
addFactor!(fg1, [:x0], IIF.Prior( MvNormal([0; 0; 0], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))
romeVis1 = BasicFactorGraphPose("DemoRobot","LocalHexVisDemo"*string(uuid4())[1:6], fg1, meanmax=:mean, poseProp = plDrawProp(0.3, 0.1, RGBA(0,1,1,0.5)))

fg2 = MetaGraphsDFG{SolverParams}(description="LG DFG",userId="user",robotId="robot",sessionId="session"*string(uuid4())[1:6],params=SolverParams())
# Add the first pose :x0
addVariable!(fg2, :x0, Pose2, labels=[:POSE])
# Add at a fixed location PriorPose2 to pin :x0 to a starting location (10,10, pi/4)
addFactor!(fg2, [:x0], IIF.Prior( MvNormal([4; 0; -pi], Matrix(Diagonal([0.1;0.1;0.05].^2)) )))
romeVis2 = BasicFactorGraphPose(fg2, meanmax=:mean)


# Create AbstractVarsVis container to hold different visualizers
visdatasets = AbstractAmphitheatre[romeVis1, romeVis2]

vis, vistask = visualize(visdatasets, start_browser=true)


##
for session in visdatasets
    fg = session.fg

    # Drive around in a hexagon
    for i in 0:5
        psym = Symbol("x$i")
        nsym = Symbol("x$(i+1)")
        addVariable!(fg, nsym, RoME.Pose2, labels=[:POSE])
        pp = Pose2Pose2(MvNormal([1.0;0;pi/3+0.02], Matrix(Diagonal([0.01;0.01;0.1].^2))))
        addFactor!(fg, [psym;nsym], pp )
        sleep(0.5)
    end

    # Add landmarks with Bearing range measurements
    addVariable!(fg, :l1, RoME.Point2, labels=[:LANDMARK])
    p2br = Pose2Point2BearingRange(IIF.Normal(0,0.01),IIF.Normal(2.0,0.1))
    addFactor!(fg, [:x6; :l1], p2br)

    # Add landmarks with Bearing range measurements
    p2br2 = Pose2Point2BearingRange(IIF.Normal(0,0.01),IIF.Normal(2.0,0.1))
    addFactor!(fg, [:x6; :l1], p2br2)
end

addFactor!(fg1, [:l1], PriorPoint2(MvNormal([2.,0], Matrix(Diagonal([0.001, 0.001].^2))) ))
addFactor!(fg2, [:l1], PriorPoint2(MvNormal([2.,0], Matrix(Diagonal([0.001, 0.001].^2))) ))

## solve
@async solveTree!(fg1)

##
@info "To stop call stopAmphiVis!()"
#
stopAmphiVis!()
##
# To restart:
vistask = Amphitheatre.restartVisualize(vis, visdatasets)
