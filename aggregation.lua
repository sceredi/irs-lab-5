-- Put your global variables here
local robot_wrapper = require("src.wrapper.robot_wrapper")
local perceptual_schemas = require("src.perceptual_schemas")
local vector = require("src.vector")
local utils = require("src.utils")

local MAX_VELOCITY = 10
local MAXRANGE = 30

local STATE = {
	STOPPED = false,
}

local PSmax = 0.99
local S = 0.01
local alpha = 0.1

local PWmin = 0.005
local W = 0.1
local beta = 0.05

local n_steps = 0
local left_v = 0
local right_v = 0

function init()
	left_v = 0
	right_v = 0
	robot_wrapper.wheels.set_velocity(left_v, right_v)
	n_steps = 0
end

-- uniform_field used to move the robot around
local function uniform_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = angle }
end

-- repulsive_field used to avoid obstacles
local function repulsive_field(strength, angle)
	local length = 0
	if strength > 0 then
		length = math.min(strength, S) / S
	else
		length = 0
	end
	return { length = length, angle = -angle }
end

-- wrapper around the perceptual schema
local function avoid_obstacles()
	local closest_prox = perceptual_schemas.closest_proximity_sensor()
	return { strength = closest_prox.length, angle = closest_prox.angle }
end

local function CountRAB()
	local number_robot_sensed = 0
	for i = 1, #robot.range_and_bearing do
		-- for each robot seen, check if it is close enough.
		if robot.range_and_bearing[i].range < MAXRANGE and robot.range_and_bearing[i].data[1] == 1 then
			number_robot_sensed = number_robot_sensed + 1
		end
	end
	return number_robot_sensed
end

-- random_walk that really isn't very random as the robot goes straight and avoids obstacles
local function random_walk()
	local obstacle_polar_val = avoid_obstacles()
	local obstacle_repulsion = repulsive_field(obstacle_polar_val.strength, obstacle_polar_val.angle)
	local go_straight = uniform_field(1, 0)
	local sum = vector.vec2_polar_sum(go_straight, obstacle_repulsion)
	local left_vel, right_vel = utils.translational_to_differential(sum.length, sum.angle)
	local maxValue = math.max(left_vel, right_vel)
	local scaleFactor = MAX_VELOCITY / maxValue
	left_vel = left_vel * scaleFactor
	right_vel = right_vel * scaleFactor

	return left_vel, right_vel
end

local function stand()
	return 0, 0
end

-- everything the robot has to do in a tick while in the wandering state
local function wandering(N)
	Ps = math.min(PSmax, S + alpha * N)
	local t = robot_wrapper.random.uniform(0, 1)
	if t <= Ps then
		STATE.STOPPED = true
		return stand()
	end
	return random_walk()
end

-- everything the robot has to do in a tick while in the standing state
local function standing(N)
	Pw = math.max(PWmin, W - beta * N)
	local t = robot_wrapper.random.uniform(0, 1)
	if t <= Pw then
		STATE.STOPPED = false
		return random_walk()
	end
	return stand()
end

-- all entrance point for the finite state automaton
local function fsa_step()
	local N = CountRAB()
	if STATE.STOPPED then
		left_v, right_v = standing(N)
	else
		left_v, right_v = wandering(N)
	end
	robot_wrapper.wheels.set_velocity(left_v, right_v)
end

function step()
	n_steps = n_steps + 1
	fsa_step()
	local state = STATE.STOPPED and 1 or 0
	robot.range_and_bearing.set_data(1, state)
end

function reset()
	left_v = MAX_VELOCITY
	right_v = 0
	robot_wrapper.wheels.set_velocity(left_v, right_v)
	n_steps = 0
end

function destroy() end
