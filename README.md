# wemos-d1-mini-Fridge
 
[Featured in this Splunk Blog Post](https://www.splunk.com/de_de/blog/tips-and-tricks/leuchtet-im-kuehlschrank-eigentlich-licht.html) 

[more information on the main project](https://github.com/SebastianWalker/ESP8266-12F-Splunk-HEC)

#software
* if fresh deployed the clientName defaults to MAC_ADDRESS and will be overwritten by first setup with the device mac address
* WiFi AP is project name + mac address to avoid duplicate SSIDs
* dashboard shows httpResponseCode from Splunk Indexer as chart
* unified sensors explained
* 

# ToDo
* make readme nice and shiny
* needs fix in documentation
* better comments in source
* fix random crash and resart of sensor
* transfer learnings back to main project

# Captain's Log :D
* came back to this after a few weeks (moved 100km to a new home)
* vs code and platformio greeted me with an error msg i didnt se before: ‘cannot find ..\framework-arduinoespressif8266\tools\sdk\libc\xtensa-lx106-elf\include‘
** solution so far: open a terminal in the project and call ‘pio platform update‘
** go to IoT framework run `npm ci`and `npx browserslist@latest --update-db`and `npm run build` once.. then build solution again
** for some reason the wifi AP did get a FairyLight_last6digitsOfMacAddr name and i could not connect to it.. after changing the AP name in code and recompile it worked again.. even after rollback of the change.. anyways maybe it worked because i deleted the build folder 

