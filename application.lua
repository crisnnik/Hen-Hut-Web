log = "" -- Keep a log in memory with a fixed size
maxLogLength = 5000 -- 5kb
doorStatus = "Unknown"
doorState = "Unknown"
fanDetail = "Unknown"
coopLightDetail = "Unknown"
runLightDetail = "Unknown"
airTemp = "Unknown";

print("Start web server")
srv=net.createServer(net.TCP)
srv:listen(80,function(conn)
    conn:on("receive",function(conn,payload)
        local request = createRequest(payload)
        local cmd = ""
        local html = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\nConnection: close\r\n\r\n"
        if request ~= null then
            cmd = string.sub(request.path, 2, string.len(request.path))
            if(cmd ~= "favicon.ico") then
                if string.find(request.path, "GetStatusAsJson") ~= nil then
                    html = "{Status:\"" .. doorStatus .. "\",Door:\"" .. doorState .. "\",Fan:\"".. fanDetail .. "\"}"
                    conn:send(html)
                    html = nil
                elseif string.find(request.path, "GetStatus") ~= nil then
                    html = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\nConnection: close\r\n\r\n"
                    html = html .. "<html><body>"
                    html = html .. "<div><h1>CP's Happy Hen Hut</h1>"
                    html = html .. "<h2>Motor: " .. doorStatus
                    html = html .. "<br />Door: " .. doorState
                    html = html .. "<br />Fan: " .. fanDetail
                    --html = html .. "<br />Coop Light: " .. coopLightDetail
                    --html = html .. "<br />Run Light: " .. runLightDetail
                    html = html .. "<br />Coop Temp: " .. airTemp .. " F</h2><br />"
                    html = html .. "<div><h1>Actions:</h2></div>"
                    html = html .. "<h2><a href=\"/OpenDoor\">Open Door</a><br><a href=\"/CloseDoor\">Close Door</a><br><a href=\"/ToggleCoop\">Toggle Coop Light</a><br><a href=\"/ToggleRun\">Toggle Run Light</a><br><a href=\"http://crisnnik.com:50001\">Chicken Cams</a><br><a href=\"/GetStatus\">Refresh Status</a></h2></html>"
                    conn:send(html)
                    html = nil --Calls to Refresh Status hung the browsers 98% of the time. This line has improved performance, still not perfect.
                elseif string.len(request.path) > 1 then
                    print(cmd) -- Send command to Arduino
                    if string.find(request.path, "AsJson") ~= nil then
                        html = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\nConnection: close\r\n\r\n"
                        html = html .. "{RequestStatus:\"OK\"}"
                    else
                        html = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\nConnection: close\r\n\r\n"
                        html = html .. "<div><h1>Command sent to Arduino...</h1></div>"
                        html = html .. "<br /><br /><h2><a href=\"/GetStatus\">Home Page</a></h2></html>"
                    end
                    conn:send(html)
                end
            else
                conn:close()
            end
        end
        request = nil
        cmd = nil
        html = nil
        collectgarbage()
    end)
    conn:on("sent",function(conn)
        conn:close()
    end)
end)

-- Request the latest variables from Arduino when the chip loads. Wait 10 seconds, to ensure Arduino loads first.
tmr.alarm(1, 10000, 0, function()
    print("SendCurrentVariablesToWifi");
end)

function lines(str)
  local t = {}
  local function helper(line) table.insert(t, line) return "" end
  helper((str:gsub("(.-)\r?\n", helper)))
  return t
end

function updateVariables(strStatus, strPosition, strFan, strCoopLight, strRunLight, strTemp)
    doorStatus = strStatus
    doorState = strPosition
    fanDetail = strFan
    coopLightDetail = strCoopLight
    runLightDetail = strRunLight
    airTemp = strTemp
end

function elSplit( value, inSplitPattern, outResults )
   if not outResults then
      outResults = { }
   end
   local theStart = 1
   local theSplitStart, theSplitEnd = string.find( value, inSplitPattern, theStart )
   while theSplitStart do
      table.insert( outResults, string.sub( value, theStart, theSplitStart-1 ) )
      theStart = theSplitEnd + 1
      theSplitStart, theSplitEnd = string.find( value, inSplitPattern, theStart )
   end
   table.insert( outResults, string.sub( value, theStart ) )
   return outResults
end

function isempty(s)
  return s == nil or s == ''
end

function createRequest(payload)
    local request = {}
    
    local splitPayload = elSplit(payload, "\r\n\r\n")
    local httpRequest = elSplit((splitPayload[1]), "\r\n")
    if not isempty((splitPayload[2])) then 
        request.content = json.decode((splitPayload[2]))
    end
    
    local splitUp = elSplit((httpRequest[1]), "%s+")
    
    request.method = (splitUp[1])
    request.path = (splitUp[2])
    request.protocal = (splitUp[3])

    local pathParts = elSplit(request.path, "/")
    local maybeId = tonumber((pathParts[table.getn(pathParts)]))

    if maybeId ~= nil then 
        request.fullPath = request.url
        request.path = string.sub(request.fullPath, 1, string.len(request.fullPath) - string.len("" .. maybeId))
        request.id = maybeId
    end
    --print(node.heap())
    httpRequest = nil
    splitUp = nil
    splitPayload = nil
    maybeId = nil
    collectgarbage()
    --print(node.heap())
    return request
end
