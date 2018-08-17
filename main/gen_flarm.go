/*
	Copyright (c) 2016-2018 Keith Tschohl / Serge Guex v1
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	flarm-nmea.go: Functions for generating FLARM-related NMEA sentences
		to communicate traffic bearing / distance to glider computers
		and to EFBs designed for European pilots.
*/

package main

import (
	//"bufio" 
	"fmt"
	"io"
	"log"
	"math"
	"net"
	"time"
	"strings"
	"strconv"
)

/*
	sendNetFLARM() is a shortcut to network.go 'sendMsg()', and will send the referenced byte slice to the UDP network port
		defined by NETWORK_FLARM_NMEA in gen_gdl90.go as a non-queueable message to be used in XCSoar. It will also queue
		the message into a channel so it can be	sent out to a TCP server.
		
		This should also allow FLARM-formatted messages to be sent over serial output, if so configured in network.go.
*/

func InBetween(i, min, max int16) bool {
	if (i >= min) && (i <= max) {
		return true
	} else {
		return false
	}
}

func sendNetFLARM(msg string) {
	if globalSettings.NetworkFLARM {
		sendMsg([]byte(msg), NETWORK_FLARM_NMEA, false) // UDP and future serial output. Traffic messages are always non-queuable -- hence 'false'.
	}
	msgchan <- msg // TCP output.
}


/*
	makeFlarmPFLAAString() creates a NMEA-formatted PFLAA string (FLARM traffic format) with checksum from the referenced
		traffic object.
*/

func makeFlarmPFLAAString(ti TrafficInfo) (msg string, valid bool) {

	/*	Format: $PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>, <ClimbRate>,<AcftType>*<checksum>
		            $PFLAA,0,-10687,-22561,-10283,1,A4F2EE,136,0,269,0.0,0*4E
			<AlarmLevel>  Decimal integer value. Range: from 0 to 3.
							Alarm level as assessed by FLARM:
							0 = no alarm (also used for no-alarm traffic information)
	   					1 = alarm, 13-18 seconds to impact
							2 = alarm, 9-12 seconds to impact
							3 = alarm, 0-8 seconds to impact
			<RelativeNorth>,<RelativeEast>,<RelativeVertical> are distances in meters. Decimal integer value. Range: from -32768 to 32767.
				For traffic without known bearing, assign estimated distance to <RelativeNorth> and leave <RelativeEast> empty
			<IDType>: 1 = official ICAO 24-bit aircraft address; 2 = stable FLARM ID (chosen by FLARM) 3 = anonymous ID, used if stealth mode is activated.
			For ADS-B traffic, we'll always pick 1.
			<ID>: 6-digit hexadecimal value (e.g. “5A77B1”) as configured in the target’s PFLAC,,ID sentence. For ADS-B targets always use reported 24-bit ICAO address.
				NOTE: Appending "!CALLSIGN" will cause compatible applications to display a callsign or tail number. 
			<Track>: Decimal integer value. Range: from 0 to 359. The target’s true ground track in degrees.
			<TurnRate>: Not used. Empty field.
			<GroundSpeed>: Decimal integer value. Range: from 0 to 32767. The target’s ground speed in m/s
			<ClimbRate>: Decimal fixed point number with one digit after the radix point (dot). Range: from -32.7 to 32.7. The target’s climb rate in m/s.
			Positive values indicate a climbing aircraft.
			<AcftType>: Hexadecimal value. Range: from 0 to F.
							Aircraft types:
							0 = unknown
							1 = glider / motor glider
							2 = tow / tug plane
							3 = helicopter / rotorcraft
							4 = skydiver
							5 = drop plane for skydivers
							6 = hang glider (hard)
							7 = paraglider (soft)
							8 = aircraft with reciprocating engine(s)
							9 = aircraft with jet/turboprop engine(s)
							A = unknown
							B = balloon
							C = airship
							D = unmanned aerial vehicle (UAV)
							E = unknown
							F = static object
	*/

	var idType, checksum uint8
	var relativeNorth, relativeEast, relativeVertical, groundSpeed int16
	var climbRate float32
	var alarmType, alarmLevel uint8
	var msgPFLAU string
	var relativeBearing float64
	var track, rEast, gSpeed, cRate string
	var alt_valid bool
	var track_valid bool
	var modec_valid bool
   
	idType = 1
 	alarmLevel = 0
	alarmType = 0
	modec_valid = false
   
	// determine distance and bearing to target
	dist, bearing, distN, distE := distRect(float64(mySituation.GPSLatitude), float64(mySituation.GPSLongitude), float64(ti.Lat), float64(ti.Lng))

	if globalSettings.DEBUG {
		log.Printf("ICAO target %X (%s) is %.1f meters away at %.1f degrees\n", ti.Icao_addr, ti.Tail, dist, bearing)
	}

//	if distN > 32767 || distN < -32767 || distE > 32767 || distE < -32767 {

	if ti.Alt > 0 {
		alt_valid = true
	}
	if ti.Track > 0 {
		track_valid = true
	}
	
	if !alt_valid {
		msg = ""
		msgPFLAU = ""
		valid = false
		if globalSettings.DEBUG {
			log.Printf("RELEVANT NO Altitude *** icao=%X (%s)\n", ti.Icao_addr, ti.Tail)
		}			
		return
		
	} else if alt_valid && ti.Position_valid && ti.Speed_valid && isGPSValid() && mySituation.GPSFixQuality > 0 { 		
		relativeNorth = int16(distN)
		relativeEast = int16(distE)
		rEast = strconv.Itoa(int(relativeEast))
		track = strconv.Itoa(int(ti.Track))
		modec_valid = false
		
		if globalSettings.DEBUG {
			log.Printf("RELEVANT ADSB *** icao=%X (%s), relN=%v, RelE=%v\n", ti.Icao_addr, ti.Tail, relativeNorth, rEast)
		}			
		
	} else if alt_valid && !ti.Position_valid && !ti.Speed_valid && !track_valid && isGPSValid() && mySituation.GPSFixQuality > 0 {

		if (ti.SignalLevel > -5) { // 463 m = 0.25 NM; 
		relativeNorth = 463
		} else if (ti.SignalLevel > -10) { // 3704 m = 2.0 NM; 
		relativeNorth = 3704
		} else if (ti.SignalLevel > -15) { // 7408 m = 4.0 NM; 
		relativeNorth = 7408
		} else if (ti.SignalLevel > -18) { // 11112 m = 6.0 NM; 
		relativeNorth = 11112
		} else if (ti.SignalLevel > -20) { // 14816 m = 8.0 NM; 
		relativeNorth = 14816
		} else if (ti.SignalLevel > -25) { // 29632 m = 16.0 NM; 
		relativeNorth = 29632
		}
		
		rEast = ""	
		dist = float64(relativeNorth)
		track = ""
		gSpeed = ""
		cRate = ""
		modec_valid = true

		if relativeNorth == 0 {
			valid = false
			modec_valid = false
			return
		} 
			
		if globalSettings.DEBUG {
			log.Printf("RELEVANT MODEC *** icao=%X (%s), alt=%v, dist=%v, cat=%v, sig=%v, modec=%v\n", ti.Icao_addr, ti.Tail, ti.Alt, dist, ti.Emitter_category, ti.SignalLevel, modec_valid)
		}
			
	} else {
		valid = false
		return			
	}
	
	altf := mySituation.BaroPressureAltitude
	
	if !isTempPressValid() { // if no pressure altitude available, use GPS altitude
		altf = float32(mySituation.GPSAltitudeMSL)
	} else if strings.Contains(ti.Tail, "F-") { // if FLARM target, use GPS altitude
		altf = float32(mySituation.GPSAltitudeMSL)
	}
 
	relativeVertical = int16(float32(ti.Alt)*0.3048 - altf*0.3048) // convert to meters


	if globalSettings.DEBUG {
		log.Printf("ModeC *** icao=%X (%s), RelVert=%d, modec=%v\n", ti.Icao_addr, ti.Tail, relativeVertical, modec_valid)
	}	
	
	 
	// check ModeC and range must be between -305m to 305m (+/- 1000ft) 
	if modec_valid && !InBetween(relativeVertical, -310, 310) {
		if globalSettings.DEBUG {
			log.Printf("ModeC *** RelVert is NOT in the range +/- 1000ft, icao=%X (%s), RelVert=%v\n", ti.Icao_addr, ti.Tail, relativeVertical)
		}
		valid=false
		return
	}

	
	// Enable alarm level for traffic within 0.5 up to 5 nautical miles and 1000' vertically. 
	// Glider pilots might want a less aggressive set of parameters, but this is a lowest-common-denominator sort of solution,
	// since relative altitude is currently calculated as GPS altitde vs traffic pressure altitude for 99% of Stratux users, and
	// since Euro airplane pilots tend to use EFBs that only support FLARM format.
	
	// There's no one setting that will please everyone. Change this if you don't like it.

	//if (dist < 926) && (relativeVertical < 304) && (relativeVertical > -304) { // 926 m = 0.5 NM; 304 = +/-1000ft
	if (dist < 926) && InBetween(relativeVertical, -304, 304) { // 926 m = 0.5 NM; 304 = +/-1000ft
		alarmLevel = 3
		alarmType = 2
		} else if (dist < 4000) && InBetween(relativeVertical, -304, 304) { // 3704 m = 2.0 NM; 304 = +/-1000ft	
		alarmLevel = 3
		alarmType = 2
		} else if (dist < 8000) && InBetween(relativeVertical, -304, 304) { // 7408 m = 4.0 NM; 304 = +/-1000ft
		alarmLevel = 2
		alarmType = 2
		} else if (dist < 12000) && InBetween(relativeVertical, -304, 304) { // 11112 m = 6.0 NM; 304 = +/-1000ft
		alarmLevel = 1
		alarmType = 2
		} else {
		alarmLevel = 0
		alarmType = 0  
		}
  
	if ti.Speed_valid {
		groundSpeed = int16(float32(ti.Speed) * 0.5144) // convert to m/s
		gSpeed = strconv.Itoa(int(groundSpeed))
		
		climbRate = float32(ti.Vvel) * 0.3048 / 60 // convert to meters per second, and limit to ±32.7
		if climbRate > 32.7 {
			climbRate = 32.7
		} else if climbRate < -32.7 {
			climbRate = -32.7
		} 
		//cRate = strconv.FormatFloat(climbRate, 'E', -1, 32)	
		cRate = fmt.Sprintf("%.1f", climbRate)
		
	} else {
		gSpeed = ""
		cRate = ""
	}

	// Set the FLARM aircraft type based on the ADS-B aircraft categories. 
	
	acType := 0
	switch ti.Emitter_category {
	case 9:
		acType = 1 // glider
	case 7:
		acType = 3 // rotorcraft
	case 1:
		acType = 8 // assume all light aircraft are piston
	case 2, 3, 4, 5, 6:
		acType = 9 // assume all heavier aircraft are jets
	default:
		acType = 0
	}

	msg = fmt.Sprintf("PFLAA,%d,%d,%s,%d,%d,%X!%s,%s,,%s,%s,%d", alarmLevel, relativeNorth, rEast, relativeVertical, idType, ti.Icao_addr, ti.Tail, track, gSpeed, cRate, acType) // extended message type; might not be compatible with all systems.

	for i := range msg {
		checksum = checksum ^ byte(msg[i])
	}
	msg = (fmt.Sprintf("$%s*%02X\r\n", msg, checksum))

// Set the FLARM aircraft ALARM. 
// syntax: PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>,<RelativeVertical>,<RelativeDistance>,<ID>

	if alarmLevel > 0 && isGPSValid() && mySituation.GPSFixQuality > 0 && !modec_valid {     
		if globalSettings.DEBUG {
		   log.Printf("FLARM Alarm: Traffic %X, AlarmType %d, AlarmLevel %d\n", ti.Icao_addr, alarmType, alarmLevel) 
		}  
		
		if ti.Bearing > 180.0 {
			relativeBearing = ti.Bearing - 360.0 
		} else if ti.Bearing < -180.0 {
			relativeBearing = ti.Bearing + 360.0 
		}    
    
		msgPFLAU = fmt.Sprintf("PFLAU,1,1,2,1,%d,%d,%d,%d,%d,%X", alarmLevel, int16(relativeBearing), alarmType, relativeVertical, int16(dist), ti.Icao_addr)
 
		checksumPFLAU := byte(0x00)
		for i := range msgPFLAU {
		checksumPFLAU = checksumPFLAU ^ byte(msgPFLAU[i])
		}
		msgPFLAU = (fmt.Sprintf("$%s*%02X\r\n", msgPFLAU, checksumPFLAU))
 
	}	else if isGPSValid() && mySituation.GPSFixQuality > 0 { 
		msgPFLAU = fmt.Sprintf("PFLAU,1,1,2,1,0,,0,,,")
		
		checksumPFLAU := byte(0x00)
		for i := range msgPFLAU {
		checksumPFLAU = checksumPFLAU ^ byte(msgPFLAU[i])
		}
		msgPFLAU = (fmt.Sprintf("$%s*%02X\r\n", msgPFLAU, checksumPFLAU))
	}  
    
  sendNetFLARM(msgPFLAU)
  
	if globalSettings.DEBUG {
		  log.Printf(msgPFLAU) 
	}	

	valid = true
	return
}

/*
	makeGPRMCString() creates a NMEA-formatted GPRMC string (GPS recommended minimum data) with checksum from the current GPS position.
		If current position is invalid, the GPRMC string will indicate no-fix.

*/

func makeGPRMCString() string {
	/*
				 RMC          Recommended Minimum sentence C
			     123519       Fix taken at 12:35:19 UTC
			     A            Status A=active or V=Void.
			     4807.038,N   Latitude 48 deg 07.038' N
			     01131.000,E  Longitude 11 deg 31.000' E
			     022.4        Speed over the ground in knots
			     084.4        Track angle in degrees True
			     230394       Date - 23rd of March 1994
			     003.1,W      Magnetic Variation
			     D				mode field (nmea 2.3 and higher)
			     *6A          The checksum data, always begins with *
		GPSLastFixSinceMidnightUTC uint32
		GPSLatitude                float32
		GPSLongitude               float32
		GPSFixQuality              uint8
		GPSGeoidSep                float32 // geoid separation, ft, MSL minus HAE (used in altitude calculation)
		GPSSatellites              uint16  // satellites used in solution
		GPSSatellitesTracked       uint16  // satellites tracked (almanac data received)
		GPSSatellitesSeen          uint16  // satellites seen (signal received)
		GPSHorizontalAccuracy      float32 // 95% confidence for horizontal position, meters.
		GPSNACp                    uint8   // NACp categories are defined in AC 20-165A
		GPSAltitudeMSL             float32 // Feet MSL
		GPSVerticalAccuracy        float32 // 95% confidence for vertical position, meters
		GPSVerticalSpeed           float32 // GPS vertical velocity, feet per second
		GPSLastFixLocalTime        time.Time
		GPSTrueCourse              uint16
		GPSGroundSpeed             uint16
		GPSLastGroundTrackTime     time.Time
	*/

	lastFix := float64(mySituation.GPSLastFixSinceMidnightUTC)
	hr := math.Floor(lastFix / 3600)
	lastFix -= 3600 * hr
	mins := math.Floor(lastFix / 60)
	sec := lastFix - mins*60

	status := "V"
	if isGPSValid() && mySituation.GPSFixQuality > 0 {
		status = "A"
	}

	lat := float64(mySituation.GPSLatitude)
	ns := "N"
	if lat < 0 {
		lat = -lat
		ns = "S"
	}

	deg := math.Floor(lat)
	min := (lat - deg) * 60
	lat = deg*100 + min

	ew := "E"
	lng := float64(mySituation.GPSLongitude)
	if lng < 0 {
		lng = -lng
		ew = "W"
	}

	deg = math.Floor(lng)
	min = (lng - deg) * 60
	lng = deg*100 + min

	gs := float32(mySituation.GPSGroundSpeed)
	trueCourse := float32(mySituation.GPSTrueCourse)
	yy, mm, dd := time.Now().UTC().Date()
	yy = yy % 100
	var magVar, mvEW string
	mode := "N"
	if mySituation.GPSFixQuality == 1 {
		mode = "A"
	} else if mySituation.GPSFixQuality == 2 {
		mode = "D"
	}

	var msg string

	if isGPSValid() {
		msg = fmt.Sprintf("GPRMC,%02.f%02.f%05.2f,%s,%010.5f,%s,%011.5f,%s,%.1f,%.1f,%02d%02d%02d,%s,%s,%s", hr, mins, sec, status, lat, ns, lng, ew, gs, trueCourse, dd, mm, yy, magVar, mvEW, mode)
	} else {
		msg = fmt.Sprintf("GPRMC,,%s,,,,,,,%02d%02d%02d,%s,%s,%s", status, dd, mm, yy, magVar, mvEW, mode) // return null lat-lng and velocity if Stratux does not have a valid GPS fix
	}

	var checksum byte
	for i := range msg {
		checksum = checksum ^ byte(msg[i])
	}
	msg = fmt.Sprintf("$%s*%X\r\n", msg, checksum)
	return msg
}

/*
	makeGPGGAstring() creates a NMEA-formatted GPGGA string (GPS fix data) with checksum from the current GPS position.
		If current position is invalid, the a GPTXT string indicating the error condition will be returned.

		This function is needed by some EFBs to generate traffic targets (for others, GPRMC is sufficient).
*/


func makeGPGGAString() string {
	/*
	 xxGGA
	 time
	 lat (degmin.mmm)
	 NS
	 long (degmin.mmm)
	 EW
	 quality
	 numSV
	 HDOP
	 alt
	 ualt
	 sep
	 uSep
	 diffAge
	 diffStation
	*/

	thisSituation := mySituation
	lastFix := float64(thisSituation.GPSLastFixSinceMidnightUTC)
	hr := math.Floor(lastFix / 3600)
	lastFix -= 3600 * hr
	mins := math.Floor(lastFix / 60)
	sec := lastFix - mins*60

	lat := float64(mySituation.GPSLatitude)
	ns := "N"
	if lat < 0 {
		lat = -lat
		ns = "S"
	}

	deg := math.Floor(lat)
	min := (lat - deg) * 60
	lat = deg*100 + min

	ew := "E"
	lng := float64(mySituation.GPSLongitude)
	if lng < 0 {
		lng = -lng
		ew = "W"
	}

	deg = math.Floor(lng)
	min = (lng - deg) * 60
	lng = deg*100 + min

	numSV := thisSituation.GPSSatellites
	if numSV > 12 { // standard messages limit satellite count to 12
		numSV = 12
	}

	//hdop := float32(thisSituation.Accuracy / 4.0)
	//if hdop < 0.7 {hdop = 0.7}
	hdop := 1.0 // hard code for now (testing)

	alt := thisSituation.GPSAltitudeMSL / 3.28084
	geoidSep := thisSituation.GPSGeoidSep / 3.28084

	var msg string

	if isGPSValid() {
		msg = fmt.Sprintf("GPGGA,%02.f%02.f%05.2f,%010.5f,%s,%011.5f,%s,%d,%d,%.2f,%.1f,M,%.1f,M,,", hr, mins, sec, lat, ns, lng, ew, thisSituation.GPSFixQuality, numSV, hdop, alt, geoidSep)
	} else {
		msg = fmt.Sprintf("GPTXT,No valid Stratux GPS position") // return text message type if no position
	}

	var checksum byte
	for i := range msg {
		checksum = checksum ^ byte(msg[i])
	}
	msg = fmt.Sprintf("$%s*%X\r\n", msg, checksum)
	return msg

}

/*******

Basic TCP server for sending NMEA messages to TCP-based (i.e. AIR Connect compatible)
software: SkyDemon, RunwayHD, etc.

Based on Andreas Krennmair's "Let's build a network application!" chat server demo
http://synflood.at/tmp/golang-slides/mrmcd2012.html#2

********/

type tcpClient struct {
	conn net.Conn
	ch   chan string
}

var msgchan chan string

func tcpNMEAListener() {
	ln, err := net.Listen("tcp", ":2000")
	if err != nil {
		fmt.Println(err)
		return
	}

	msgchan = make(chan string, 1024) // buffered channel n = 1024
	addchan := make(chan tcpClient)
	rmchan := make(chan tcpClient)

	go handleMessages(msgchan, addchan, rmchan)

	for {
		conn, err := ln.Accept()
		if err != nil {
			fmt.Println(err)
			continue
		}

		go handleConnection(conn, msgchan, addchan, rmchan)
	}
}


/*
func (c tcpClient) ReadLinesInto(ch chan<- string) {
	bufc := bufio.NewReader(c.conn)
	for {
		line, err := bufc.ReadString('\n')
		if err != nil {
			break
		}
		ch <- fmt.Sprintf("%s: %s", c.nickname, line)
	}
}
*/

func (c tcpClient) WriteLinesFrom(ch <-chan string) {
	for msg := range ch {
		_, err := io.WriteString(c.conn, msg)
		if err != nil {
			return
		}
	}
}


/*
	func handleConnection().
	 Opens the TCP connection for a given client. Behavior emulates AIR Connect device in the following ways.
	 
	 1. Send the string "PASS?" to clients upon opening the connection. This prompts the client software to send a PIN code.
	 2. [Currently ignored since it isn't needed, and because this removes the need to conduct a read] Wait for the client to provide a valid 4-digit code
	 3. Send acknowledgment "AOK" and add register this connection to send data
	 4. Upon a client disconnect, deregister the client.
*/


func handleConnection(c net.Conn, msgchan chan<- string, addchan chan<- tcpClient, rmchan chan<- tcpClient) {
	//bufc := bufio.NewReader(c)
	defer c.Close()
	client := tcpClient{
		conn: c,
		ch:   make(chan string),
	}
	io.WriteString(c, "PASS?")

	// disabling passcode checks. RunwayHD and SkyDemon don't send CR / LF, and PIN check is something else that can go wrong.
	//time.Sleep(100 * time.Millisecond)

	//code, _, _ := bufc.ReadLine()
	//log.Printf("Passcode entry was %v\n",code)

	//passcode := ""
	/*for passcode != "6000" {
		io.WriteString(c, "PASS?")
		code, _, err := bufc.ReadLine()

		if err != nil {
			log.Printf("Error scanning passcode from client %s: %s\n",c.RemoteAddr(), err)
			continue
		}
		passcode = string(code)
		log.Printf("Received passcode %s from client %s\n", passcode, c.RemoteAddr())
	}
	*/
	io.WriteString(c, "AOK") // correct passcode received; continue to writes
	log.Printf("Correct passcode on client %s. Unlocking.\n", c.RemoteAddr())
	// Register user
	addchan <- client
	defer func() {
		log.Printf("Connection from %s closed.\n", c.RemoteAddr())
		rmchan <- client
	}()

	// I/O
	//go client.ReadLinesInto(msgchan)  //treating the port as read-only once it's opened
	client.WriteLinesFrom(client.ch)
}

func handleMessages(msgchan <-chan string, addchan <-chan tcpClient, rmchan <-chan tcpClient) {
	clients := make(map[net.Conn]chan<- string)

	for {
		select {
		case msg := <-msgchan:
			if globalSettings.DEBUG {
				log.Printf("New message: %s", msg)
			}
			for _, ch := range clients {
				go func(mch chan<- string) { mch <- msg }(ch)
			}
		case client := <-addchan:
			log.Printf("New client: %v\n", client.conn)
			clients[client.conn] = client.ch
		case client := <-rmchan:
			log.Printf("Client disconnects: %v\n", client.conn)
			delete(clients, client.conn)
		}
	}
}
