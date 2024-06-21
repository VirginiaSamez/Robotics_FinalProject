function sysCall_init()
    robot = sim.getObject('.')
    
    -- Get motors
    motorLeft = sim.getObject("./leftMotor")
    motorRight = sim.getObject("./rightMotor")
    
    -- Set up the maze grid
    gridSizeX = 0.50  -- width
    gridSizeY = 0.85 -- length

    -- Simple maze map
    maze = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1}, 
        {1, 0, 0, 0, 0, 0, 0, 0, 1}, 
        {1, 0, 1, 0, 1, 1, 1, 1, 1}, 
        {1, 0, 1, 0, 1, 0, 0, 0, 1}, 
        {1, 1, 1, 0, 1, 0, 1, 0, 1}, 
        {1, 0, 0, 0, 0, 0, 1, 0, 1}, 
        {1, 1, 1, 1, 1, 1, 1, 0, 1}
    }
    start = {1, 1}  -- Start position
    goal = {6, 7}   -- Goal (exit) position

    -- A*
    path = aStar(maze, start, goal)
    if path then
        print("Path found!")
        for i, node in ipairs(path) do
            print(node[1], node[2])
        end
    else
        print("No path found.")
    end
    
    -- Convert path to waypoints (world coordinates)
    waypoints = {}
    for i, node in ipairs(path) do
        x = (node[2] - 1) * gridSizeX + gridSizeX / 2
        y = (node[1] - 1) * gridSizeY + gridSizeY / 2
        table.insert(waypoints, {x, y})
    end
    
    -- Set initial waypoint index
    waypointIndex = 1
    
    -- Initialize PID parameters
    Kp = 1.0
    Ki = 0.0
    Kd = 0.1
    prevError = 0
    integral = 0

    -- Speed parameters
    maxSpeed = 2
    minSpeed = 0.1

    -- Safety margin
    safetyMargin = 0.2

    -- Get initial position and orientation
    local initialPosition = sim.getObjectPosition(robot, -1)
    local initialOrientation = sim.getObjectOrientation(robot, -1)
    -- print("Initial position: ", initialPosition)
    -- print("Initial orientation: ", initialOrientation)
end

function aStar(maze, start, goal)
     -- Manhattan distance heuristic
    local function heuristic(a, b)
        return math.abs(a[1] - b[1]) + math.abs(a[2] - b[2])
    end

    -- Check if a node is a valid move
    local function isValid(node)
        local row, col = node[1], node[2]
        return row > 0 and row <= #maze and col > 0 and col <= #maze[1] and maze[row][col] == 0
    end

    -- Get neighbors of a node
    local function neighbors(node)
        local n = {}
        local row, col = node[1], node[2]
        local directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
        for _, d in ipairs(directions) do
            local neighbor = {row + d[1], col + d[2]}
            if isValid(neighbor) then
                table.insert(n, neighbor)
            end
        end
        return n
    end

    -- Initialize A* vars
    local openSet = {start}
    local cameFrom = {}
    local gScore = {}
    local fScore = {}

    gScore[start] = 0
    fScore[start] = heuristic(start, goal)

    while #openSet > 0 do
        table.sort(openSet, function(a, b) return fScore[a] < fScore[b] end)
        local current = table.remove(openSet, 1)

        if current[1] == goal[1] and current[2] == goal[2] then
            local path = {}
            while current do
                table.insert(path, 1, current)
                current = cameFrom[current]
            end
            return path
        end

        for _, neighbor in ipairs(neighbors(current)) do
            local tentative_gScore = gScore[current] + 1
            if not gScore[neighbor] or tentative_gScore < gScore[neighbor] then
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal)
                if not table.contains(openSet, neighbor) then
                    table.insert(openSet, neighbor)
                end
            end
        end
    end

    return nil  -- No path found
end

function table.contains(table, element)
    for _, value in pairs(table) do
        if value[1] == element[1] and value[2] == element[2] then
            return true
        end
    end
    return false
end

function sysCall_actuation()
    if waypointIndex <= #waypoints then
        -- Get the current waypoint
        local target = waypoints[waypointIndex]
        
        -- Get robot's current position
        local position = sim.getObjectPosition(robot, -1)
        local angle = sim.getObjectOrientation(robot, -1)[3]

        -- Calculate direction vector
        local dx = target[1] - position[1]
        local dy = target[2] - position[2]

        -- Calculate the desired angle to the target
        local targetAngle = math.atan2(dy, dx)
        
        -- Calculate the difference between current and target angle
        local angleDiff = targetAngle - angle
        angleDiff = (angleDiff + math.pi) % (2 * math.pi) - math.pi

        -- Debug: Print detailed information
        -- print("Current position: ", position)
        -- print("Current angle: ", angle)
        -- print("Target position: ", target)
        -- print("Angle difference: ", angleDiff)
        
        -- PID Control for turning
        local error = angleDiff
        integral = integral + error
        local derivative = error - prevError
        prevError = error
        
        local turnRate = Kp * error + Ki * integral + Kd * derivative
        
        -- Adaptive speed control
        local distance = math.sqrt(dx * dx + dy * dy)
        local speed = math.max(minSpeed, maxSpeed * (1 - math.abs(turnRate) / math.pi))

        -- Set motor velocities
        local leftSpeed = speed - turnRate
        local rightSpeed = speed + turnRate

        -- Debug: Print motor speeds and waypoint info
        -- print("Left speed: ", leftSpeed, " Right speed: ", rightSpeed)
        -- print("Current waypoint: ", waypointIndex, "/", #waypoints)
        -- print("Distance to waypoint: ", distance)
        
        sim.setJointTargetVelocity(motorLeft, leftSpeed)
        sim.setJointTargetVelocity(motorRight, rightSpeed)
        
        -- Check if we reached the waypoint
        if distance < 0.3 + safetyMargin then
            waypointIndex = waypointIndex + 1
        end
    else
        -- Stop the robot when waypoints are reached
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        print("Goal reached!")
    end
end