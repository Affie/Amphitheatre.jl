@deprecate visPointCloudOnPose!(args...;kwargs...) visPointCloud!(args...;kwargs...)

"""
    $(SIGNATURES)
Draw point cloud on [vis].
xTc -> object(vis) to camera transform, # Use material to set size of particles or other propeties
"""
function visPointCloud!(vis::Visualizer, pointcloud::PointCloud; xTc::SE3 = SE3([0,0,0],I), material = PointsMaterial(size=0.005) )::Nothing
	setobject!(vis, pointcloud, material)
	trans = Translation(xTc.t[1], xTc.t[2], xTc.t[3])∘LinearMap(UnitQuaternion(xTc.R.R))
	settransform!(vis, trans)
	return nothing
end
