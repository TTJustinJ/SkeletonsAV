module Perception

using VehicleSim
using LinearAlgebra
using StaticArrays

export TrackedObject, process_camera_measurement

# TrackedObject: represents a detected obstacle.
# Fields:
#   id        : a unique identifier (here, simply the index in the measurement)
#   position  : candidate 3D point in the camera coordinate frame [X, Y, Z]
#   bbox      : the original bounding box in pixel coordinates [top, left, bottom, right]
#   velocity  : a candidate velocity (set to zeros in this skeleton)
struct TrackedObject
    id::Int
    position::Vector{Float64}  # Candidate 3D point in camera frame.
    bbox::Vector{Int}          # [top, left, bottom, right]
    velocity::Vector{Float64}  # For now, set to zeros.
end

# Typical vehicle height (meters) used for depth estimation.
const VEHICLE_HEIGHT = 1.5

"""
    process_camera_measurement(cam_meas::VehicleSim.CameraMeasurement)

Projects each bounding box in the CameraMeasurement from pixel coordinates into a candidate 3D point in the camera frame
using the pinhole camera model. No transformation to a global frame is performed.
It then packages each detection as a TrackedObject with the candidate point and bounding box.
"""
function process_camera_measurement(cam_meas::VehicleSim.CameraMeasurement)
    tracked_objects = TrackedObject[]
    
    # Get camera intrinsics.
    focal_length = cam_meas.focal_length
    pixel_len = cam_meas.pixel_length
    image_width = cam_meas.image_width
    image_height = cam_meas.image_height
    # Compute image center (assuming pixel coordinates start at (1,1) top-left).
    image_center = [ (image_width + 1) / 2, (image_height + 1) / 2 ]
    
    # For each bounding box.
    for (i, bbox) in enumerate(cam_meas.bounding_boxes)
        top, left, bottom, right = bbox
        # Compute center pixel.
        cx = (left + right) / 2
        cy = (top + bottom) / 2
        # Compute the bounding box height in pixels.
        pixel_height = bottom - top
        if pixel_height == 0
            continue  # Avoid division-by-zero.
        end
        
        # Estimate depth using the pinhole camera model:
        #     depth ≈ (vehicle height * focal_length) / (pixel_height * pixel_len)
        depth = VEHICLE_HEIGHT * focal_length / (pixel_height * pixel_len)
        
        # Compute candidate 3D point in camera frame.
        X_cam = (cx - image_center[1]) * depth * pixel_len / focal_length
        Y_cam = (cy - image_center[2]) * depth * pixel_len / focal_length
        Z_cam = depth
        candidate_point = [X_cam, Y_cam, Z_cam]
        
        # Create a tracked object.
        obj = TrackedObject(i, candidate_point, [top, left, bottom, right], [0.0, 0.0, 0.0])
        push!(tracked_objects, obj)
    end
    return tracked_objects
end

end  # module Perception
