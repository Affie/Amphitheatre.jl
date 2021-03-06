module Amphitheatre

using MeshCat
using GeometryBasics
using Colors
using PlotUtils
using Sockets: @ip_str, IPAddr, IPv4, IPv6
#
using DistributedFactorGraphs

# using Mongoc
# using JSON2
# using Base64

#
using DocStringExtensions
using TransformUtils
using CoordinateTransformations, Rotations
using StaticArrays

const CTs = CoordinateTransformations
const TUs = TransformUtils

export
	BasicFactorGraphPose,
	AbstractAmphitheatre,
	visualize,
	Visualizer,
	startMeshCatVisualizer,
	stopAmphiVis!,
#Common visse
	visPose!,
	visPoint!,
#camera
	CameraModel,
#DepthImages
	cloudFromDepthImageClampZ,
	cloudFromDepthImage,
#Amphis
	TagkTl,
	AbstractAmphitheatre,
	plDrawProp,
	BasicFactorGraphPose,
	BasicGraffPose,
	TagOnPose,
	PCloudFactorGraphPose,
	GraffTagOnPose,
	GraffCloudOnPose,
	#re-export
	RGBA,
	ColorGradient, cgrad,
	@ip_str, IPAddr, IPv4, IPv6


include("Common/CameraModel.jl")
include("Common/DepthImages.jl")

include("common.jl")
include("amphis.jl")
include("pointclouds.jl")
# TODO include("pointcloudAmphis.jl")
include("reprojectAmphis.jl") #TODO


global runAmphi = true

function stopAmphiVis!()
	global runAmphi
  	runAmphi = false
  	nothing
end


"""
    $(SIGNATURES)
Initialize empty visualizer window with home axis.  New browser window will be opened based on `start_browser=true`.
"""
function startMeshCatVisualizer(;host=ip"127.0.0.1",
								 start_browser::Bool=true,
                                 draworigin::Bool=true,
                                 originscale::Float64=1.0,
								 openMux::Bool=true)

    viz = MeshCat.Visualizer(MeshCat.CoreVisualizer(host), ["amphitheatre"])
    if draworigin
      setobject!( viz[:origin], Triad(originscale) )
    end

    # open a new browser tab if required
	openMux && open(viz, start_browser=start_browser)

    return viz
end


"""
   $(SIGNATURES)
High level interface to launch webserver process that draws a factor_graph_vis_type <: AbstractAmphitheatre, using Three.js and MeshCat.jl.
User factor_graph_vis_type should provide a visualize!(vis::Visualizer, factor_graph_vis_variables::T<:AbstractAmphitheatre ) function.
"""
function visualize(visdatasets::Vector{AbstractAmphitheatre};
					trans=Translation(0.0,0.0,0.0),
					quat::Rotations.UnitQuaternion=UnitQuaternion(1.0,0.0,0.0,0.0),
				   	host=ip"127.0.0.1",
					start_browser::Bool=true,
				    draworigin::Bool=true,
				    originscale::Float64=1.0,
					openMux::Bool=true)
    #
    global runAmphi

    runAmphi = true

    # the visualizer object itself
    vis = startMeshCatVisualizer(host=host,
								 start_browser=start_browser,
								 draworigin=draworigin,
								 originscale=originscale,
								 openMux=openMux)
	setGlobalDrawTransform!(vis, trans=trans, quat=quat)

    # run the visualization loop #TODO add is task done to avoid multiple tasks getting lost
    runAphiTask = @async begin
		while runAmphi
			# iterate through all datasets #vir wat staan rose_fgl?
			for rose_fgl in visdatasets
				# each dataset should provide an visualize function
				visualize!(vis, rose_fgl)
			end
			# take a break and repeat
			sleep(1)
		end
		@info "visualize is finalizing."
	end

    return vis, runAphiTask
end

function restartVisualize(vis, visdatasets)
	global runAmphi
	runAmphi = true
	runAphiTask = @async begin
		while runAmphi
			# iterate through all datasets #vir wat staan rose_fgl?
			for rose_fgl in visdatasets
				# each dataset should provide an visualize function
				visualize!(vis, rose_fgl)
			end
			# take a break and repeat
			sleep(1)
		end
		@info "visualize is finalizing."
	end
	return runAphiTask
end

end
