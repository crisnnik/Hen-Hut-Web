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
                    html = html .. "<!doctype html>"
                    html = html .. "<html class='no-js' lang='en'>"
                    html = html .. "    <head>"
                    html = html .. "        <link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/foundation/6.4.3/css/foundation.min.css' />"
                    html = html .. "        <meta charset='utf-8' />"
                    html = html .. "        <meta name='viewport' content='width=device-width, initial-scale=1.0' />"
                    html = html .. "        <meta http-equiv='refresh' content='60' >"
                    html = html .. "        <title>TEST - Peck's Happy Hen Hut</title>"
                    html = html .. "    </head>"
                    html = html .. "    <body>"
                    html = html .. "        <div class='top-bar'>"
                    html = html .. "            <div class='top-bar-left menu-text'>TEST - Peck's Happy Hen Hut</div>"
                    html = html .. "        </div>"
                    html = html .. "        <div class='grid-x grid-margin-y grid-padding-y grid-padding-x'>"
                    html = html .. "            <div class='cell'>"
                    html = html .. "                <img alt='header_img' class='float-center' src='https://blog.imgur.com/wp-content/uploads/2016/12/chicken.jpg' />"
                    html = html .. "            </div>"
                    html = html .. "        </div>"
                    html = html .. "        <div class='grid-x grid-margin-x grid-padding-x'>"
                    html = html .. "            <div class='small-12 medium-6 large-6 cell'>"
                    html = html .. "                <h2>TEST - Peck's Happy Hen Hut</h2>"
                    html = html .. "                <ul class='no-bullet'>"
                    html = html .. "                    <li>Motor: " .. doorStatus .. " </li>"
                    html = html .. "                    <li>Door: " .. doorState .. " </li>"
                    html = html .. "                    <li>Fan: " .. fanDetail .. " </li>"
                    html = html .. "                    <li>Coop Light: " .. coopLightDetail .. " </li>"
                    html = html .. "                    <li>Run Light: " .. runLightDetail .. " </li>"
                    html = html .. "                    <li>Coop Temp: " .. airTemp .. "°F</li>"
                    html = html .. "                </ul>"
                    html = html .. "            </div>"
                    html = html .. "            <div class='small-12 medium-6 large-6 cell'>"
                    html = html .. "                <h2>Actions</h2>"
                    html = html .. "                <div class='stacked button-group'>"
                    html = html .. "                    <a class='button' href='/OpenDoor'>Open Door</a>"
                    html = html .. "                    <a class='button' href='/CloseDoor'>Close Door</a>"
                    html = html .. "                    <a class='button' href='/ToggleCoop'>Toggle Coop Light</a>"
                    html = html .. "                    <a class='button' href='/ToggleRun'>Toggle Run Light</a>"
                    html = html .. "                    <a class='button' href='http://crisnnik.com:50001'>Chicken Cams</a>"
                    html = html .. "                    <a class='button' href='/GetStatus'>Refresh Status</a>"
                    html = html .. "                </div>"
                    html = html .. "            </div>"
                    html = html .. "        </div>"
                    html = html .. "    </body>"
                    html = html .. "</html>"
                    conn:send(html)
                    html = nil --Calls to Refresh Status hung the browsers 98% of the time. This line has improved performance, still not perfect.
                elseif string.len(request.path) > 1 then
                    print(cmd) -- Send command to Arduino
                    if string.find(request.path, "AsJson") ~= nil then
                        html = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\nConnection: close\r\n\r\n"
                        html = html .. "{RequestStatus:'OK'}"
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
