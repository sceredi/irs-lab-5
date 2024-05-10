local vector = require("src.vector")
local robot_wrapper = require("src.wrapper.robot_wrapper")
local M = {}

function M.sum_proximity_sensors()
	local sum = { length = 0, angle = 0 }
	for i = 1, 24 do
		sum = vector.vec2_polar_sum(sum, {
			length = robot_wrapper.get_proximity_sensor_readings()[i].value,
			angle = robot_wrapper.get_proximity_sensor_readings()[i].angle,
		})
	end
	return sum
end

function M.closest_proximity_sensor()
	local closest = robot_wrapper.get_proximity_sensor_readings()[1]
	for i = 2, 24 do
		if i <= 6 or i >= 19 then
			if robot_wrapper.get_proximity_sensor_readings()[i].value > closest.value then
				closest = robot_wrapper.get_proximity_sensor_readings()[i]
			end
		end
	end
	return { length = closest.value, angle = closest.angle }
end

return M
