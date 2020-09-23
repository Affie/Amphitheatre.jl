#all types shoud inheret from AbstractAmphitheatre and provide a visualize! function
abstract type AbstractAmphitheatre end

#pose landmark draw property #TODO Move to core?
mutable struct plDrawProp
	scale::Float64
	sphereScale::Float64
	color::RGBA
end

plDrawProp() = plDrawProp(0.3, 0.1, RGBA())

# ============================================================
# --------------------BasicFactorGraphPose--------------------
# ============================================================

#TODO All DFGs now has robotId and sessionId, change to use that
struct BasicFactorGraphPose <: AbstractAmphitheatre
	robotId::String
	sessionId::String
	fg::AbstractDFG

	nodes::Dict{Symbol, AbstractPointPose} #poseId, Softtype
	meanmax::Symbol
	zoffset::Float64
	drawPath::Bool
	#pose drawing propeties
	poseProp::plDrawProp
	#landmark drawing propeties
	landmarkProp::plDrawProp
end


"""
    $(SIGNATURES)
Basic visualizer object to draw poses and landmarks.
"""
function BasicFactorGraphPose(robotId::String, sessionId::String, fg::AbstractDFG;
							  meanmax::Symbol=:max,
						      zoffset::Float64=0.0,
							  drawPath::Bool=false,
							  poseProp::plDrawProp = plDrawProp(0.15, 0.05, RGBA(1,1,0,0.5)),
							  landmarkProp::plDrawProp = plDrawProp(0.2, 0.1, RGBA(0,1,0,0.5)))

    return BasicFactorGraphPose(robotId, sessionId, fg, Dict{Symbol,AbstractPointPose}(),
							    meanmax,
							    zoffset,
							    drawPath,
							    poseProp,
							    landmarkProp)
end

function BasicFactorGraphPose(fg::AbstractDFG;
							  meanmax::Symbol=:max,
						      zoffset::Float64=0.0,
							  drawPath::Bool=false,
							  poseProp::plDrawProp = plDrawProp(0.15, 0.05, RGBA(1,1,0,0.5)),
							  landmarkProp::plDrawProp = plDrawProp(0.2, 0.1, RGBA(0,1,0,0.5)))

    return BasicFactorGraphPose(fg.robotId, fg.sessionId, fg, Dict{Symbol,AbstractPointPose}(),
							    meanmax,
							    zoffset,
							    drawPath,
							    poseProp,
							    landmarkProp)
end


"""
    $(SIGNATURES)
Basic visualizer object visualize! function.
"""
function visualize!(vis::Visualizer, basicfg::BasicFactorGraphPose)::Nothing
	#TODO maybe improve this function to lower memmory allocations

	# TODO
	solveKey = :default

	fg = basicfg.fg

	robotId = basicfg.robotId
	sessionId = basicfg.sessionId

	# get all variables
    vars = getVariables(fg)


	basicfg.drawPath && (trackPoints = Point3f0[])
    # update the variable point-estimate cache
    for v in vars
		vsym = v.label

		ppeDict = DistributedFactorGraphs.getPPEDict(v)
		if haskey(ppeDict, solveKey)
			#TODO find a better way to use PPE
			# with this RoME pose and ampi pose has to be kept consistent.
			#TODO maybe use timestamp?
			# varEst = ppe[basicfg.meanmax]
			#TODO meanmax is ignored an suggested used for now
			sugPPE = getSuggestedPPE(ppeDict[solveKey])

			typestr = split(typeof(getSofttype(v)) |> string, ".")[end]
			typesym = Symbol(typestr)
			nodef = getfield(Amphitheatre, typesym)

			nodestruct = nodef(sugPPE...)
		else
			continue
		end
		# if ppe === nothing
	    #     # get vertex and estimate from the factor graph object
	    #     X = getKDE(v)

	    #     xmx = basicfg.meanmax == :max ? getKDEMax(X) : getKDEMean(X)
		# 	# get the variable type
	    #     typestr = split(solverData(v).softtype |> typeof |> string, ".")[end]
		# 	typesym = Symbol(typestr)

		# 	nodef = getfield(Amphitheatre, typesym)

		# 	#NOTE make sure storage order and softtypes are always the same
		# 	nodestruct = nodef(xmx...)

		# else
		# 	#TODO find a better way to use VariableEstimate
		# 	# with this RoME pose and ampi pose has to be kept consistent.
		# 	#TODO maybe use timestamp?
		# 	varEst = ppe[basicfg.meanmax]

		# 	typestr = split(softtype(v) |> string, ".")[end]
		# 	typesym = Symbol(typestr)
		# 	nodef = getfield(Amphitheatre, typesym)

		# 	nodestruct = nodef(varEst.estimate...)
		# end



		if in(:POSE, v.tags)
			groupsym = :poses
			drawProp = basicfg.poseProp
		elseif in(:LANDMARK, v.tags)
			groupsym = :landmarks
			drawProp = basicfg.landmarkProp

		# TODO remove? assume l.. is landmark and x.. is pose
		elseif string(vsym)[1] == 'l'
			groupsym = :landmarks
			drawProp = basicfg.landmarkProp
		elseif string(vsym)[1] == 'x'
			groupsym = :poses
			drawProp = basicfg.poseProp
		elseif in(:VARIABLE, v.tags)
			@warn "Unknown symbol encountered $vsym"
			groupsym = :variables
		end


		isnewnode = !haskey(basicfg.nodes, vsym)
		if isnewnode
			push!(basicfg.nodes, vsym=>nodestruct)
		else
			basicfg.nodes[vsym] = nodestruct
		end

		visNode!(vis[robotId][sessionId][groupsym][vsym], nodestruct, isnewnode,
				 zoffset = basicfg.zoffset,
				 scale = drawProp.scale,
				 sphereScale = drawProp.sphereScale,
				 color = drawProp.color)

		#FIXME this is bad, but just testing feature TODO figure out how to do it properly
		basicfg.drawPath && groupsym == :poses && push!(trackPoints, Point3f0(xmx[1],xmx[2],0.0))

	# sleep(0.1)

    end

	basicfg.drawPath && setobject!(vis[robotId][sessionId][:track], Object(PointCloud(trackPoints), LineBasicMaterial(), "Line"))

    return nothing
end
