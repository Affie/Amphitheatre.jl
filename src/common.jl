
abstract type AbstractPointPose end
#NOTE names should match soft type names with, eg. Pose2 and Point2

# TODO look at replacing with traits

mutable struct Pose2{T<:AbstractFloat} <: AbstractPointPose
	x::T
	y::T
	θ::T
end

mutable struct Point2{T<:AbstractFloat} <: AbstractPointPose
	x::T
	y::T
end

mutable struct Point3{T<:AbstractFloat} <: AbstractPointPose
	x::T
	y::T
	z::T
end

"""
    $(SIGNATURES)
Set the draw transform for the meshcat root object.

Example:
```julia
# default is Z up
# make Z down
setGlobalDrawTransform!(vis, quat=UnitQuaternion(0.0,1.0,0.0,0.0))
```
"""
function setGlobalDrawTransform!(vis::Visualizer;trans=Translation(0.0,0.0,0.0), quat::Rotations.UnitQuaternion=UnitQuaternion(1.0,0.0,0.0,0.0))
	tform = trans ∘ LinearMap(quat)
	settransform!(vis, tform)
end

"""
    $(SIGNATURES)
Basic visualize function for a pose.
"""
function visPose!(vis::Visualizer,
				  tform::AbstractAffineMap,
				  updateonly::Bool=false;
				  scale::Float64=0.2,
				  sphereScale::Float64=0.05,
				  color::RGBA=RGBA(1., 1, 0, 0.5))
	if !updateonly
		setobject!(vis, Triad(scale))
		if sphereScale > 0.0
			sphere = GeometryBasics.HyperSphere(GeometryBasics.Point(0., 0, 0), sphereScale)
			matcolor = MeshPhongMaterial(color=color)
			setobject!(vis[:bal], sphere, matcolor)
		end
	end
	settransform!(vis, tform)
	nothing
end

"""
    $(SIGNATURES)
Basic visualize function for a point.
"""
function visPoint!(vis::Visualizer,
				   tform::AbstractAffineMap,
				   updateonly::Bool=false;
				   scale::Float64=0.1, #unused but needed for uniformity #TODO maybe replace params with dictionary?
				   sphereScale::Float64=0.1,
				   color::RGBA=RGBA(0., 1, 0, 0.5))

	if !updateonly
		sphere = GeometryBasics.HyperSphere(GeometryBasics.Point(0., 0, 0), sphereScale)
        matcolor = MeshPhongMaterial(color=color)
        setobject!(vis, sphere, matcolor)
	end
	settransform!(vis, tform)
	nothing
end


"""
    $(SIGNATURES)
Node Visualization functions.
"""
function visNode!(vis::Visualizer,
	              pose::Pose2,
				  updateonly::Bool=false;
				  zoffset::Float64=0.0,
				  kwargs...)::Nothing
				  # scale::Float64=0.2,
				  # sphereScale::Float64=0.05,
				  # color::RGBA=RGBA(1., 1, 0, 0.5))::Nothing

	tf = Translation(pose.x, pose.y, zoffset) ∘ LinearMap(Rotations.RotZ(pose.θ))
	visPose!(vis, tf, updateonly; kwargs...)
					  # scale=scale,
					  # sphereScale=sphereScale,
					  # color=color)
end

function visNode!(vis::Visualizer,
            	  point::Point2,
				  updateonly::Bool=false;
                  zoffset::Float64=0.0,
				  kwargs...)::Nothing
				  # scale::Float64=0.1, #TODO i tried with kwargs...), but missing something
				  # sphereScale::Float64=0.1,
				  # color::RGBA=RGBA(0., 1, 0, 0.5))::Nothing

	tf = Translation(point.x, point.y, zoffset)
	visPoint!(vis, tf, updateonly; kwargs...)
					   # scale=scale,
					   # sphereScale=sphereScale,
					   # color=color)
end

function visNode!(vis::Visualizer,
            	  point::Point3,
				  updateonly::Bool=false;
				  zoffset::Float64=0.0,
				  kwargs...)::Nothing
				  # scale::Float64=0.1, #unused but needed for uniformity #TODO maybe replace params with dictionary?
				  # sphereScale::Float64=0.1,
				  # color::RGBA=RGBA(0., 1, 0, 0.5))::Nothing

	tf = Translation(point...)
	visPoint!(vis, tf, updateonly; kwargs...)
					   # scale=scale,
					   # sphereScale=sphereScale,
					   # color=color)
end
