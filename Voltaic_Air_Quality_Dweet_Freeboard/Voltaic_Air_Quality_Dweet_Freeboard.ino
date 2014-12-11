  /*99999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999
  
  Solar-Powered Air Quality Particle Sensor
  
  Author: Nicholas Johnson
  Corporate Sponsor: Voltaic Systems Inc. (http://www.voltaicsystems.com/ )
  Special Thanks To: Tony Kauffmann (Voltaic Systems), Chris Nafis (howmuchsnow.com), Tony Dicola (Adafruit.com)
  
  This project records dust particulates in the air and posts them to the web in real time to 
  Dweet.io. The system will be powere by a Voltaic USB battery and recharged with solar
  panels, therefore it must consume as little power as possible. To accomplish this, the Arduino 
  is put into Sleep Mode to conserve power between sensor readings.
  
  Materials:
  Shinyei PPD442 Dust Sensor 
  Arduino Uno
  GSM Shield
  Voltaic V44 USB Battery 
  Two 9W Voltaic Solar Panels
  
  Disclaimer: This project is for Solar Powered Microcontroller demonstrations only. The Shinyei Dust 
  Sensor has not been calibrated in parallel with proper test equipment, therefore the literal data
  readings can not be trusted at face value. Sensor values uploaded to Dweet are for qualitative purposes 
  only for comparisons between multiple locations, but the literal pollution may be different than what is 
  presented.
  
  99999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999*/
  
  // Include the GSM library
  #include <GSM.h>
  
  /*_________ALL THE GLOBAL SLEEP MODE REQUIREMENTS__________*/
  
  // Sleep Mode Libraries
  #include <SPI.h>
  #include <avr/sleep.h>
  #include <avr/power.h>
  #include <avr/wdt.h>
  
  // Number of times to sleep (for 8 seconds each) before the Arduino wakes up
  // ex. 30 iterations = 240 seconds = 6 minutes 
  #define MAX_SLEEP_ITERATIONS   30
  
  // Internal state used by the sketch.
  int sleepIterations = 0;
  //uint32_t ip;
  volatile bool watchdogActivated = false;
  
  /*___________END SLEEP MODE REQUIREMENTS___________*/
  
  /*___________Xively / GSM Shield Requirements________*/
  // APN data - this is unique to the carrier
  //#define GPRS_APN "wap.cingular" // AT&T
  #define GPRS_APN "epc.tmobile.com" // T-Mobile
  #define GPRS_LOGIN "admin" // this is not required by the carrier but by the library
  #define GPRS_PASSWORD "password" // ditto
  #define PINNUMBER ""
  
  // initialize the library instance
  //GSM gsm;
  GSMClient client;
  GPRS gprs;
  GSM gsmAccess(false); // switch to true for debugging
  
  char server[] = "dweet.io";
  #define THINGNAME "INCLUDE THINGNAME HERE"  // Don't use spaces 
  
  /*___________End Xively / GSM Shield Requirements________*/
  
  boolean lastConnected = false;
  
  /*___________Dust Particle Sensor Variables______________
  Understanding the Sensor: Dust/Pollution particles are counted over a fixed 
  period of time and averaged over that time interval. There are two sensor
  channels available simultaneous, one for particles 1-10 microns in diameter, 
  and another for particles 2.5-10 microns in diameter ("Coarse Particles"). 
  Particles counted from the second channel can be subtracted from the first to 
  determine "Fine Particles" (the truly dangerous pollutants).        */
  
  // Variables for Channel 1 (1-10 micrometer particles)
  int channel_1 = 8; // Yellow Wire
  unsigned long triggerOn1;
  unsigned long triggerOff1;
  unsigned long pulseLength1;
  unsigned long totalDuration1 = 0;
  float ratio1 = 0;
  float count1 = 0;
  boolean valueP1 = HIGH;
  boolean trigger1 = false;
  
  // Variables for Channel 2 (2.5-10 micrometer particles)
  int channel_2 = 9; // Blue Wire
  unsigned long triggerOn2;
  unsigned long triggerOff2;
  unsigned long pulseLength2;
  unsigned long totalDuration2 = 0;
  float ratio2 = 0;
  float count2 = 0;
  boolean valueP2 = HIGH;
  boolean trigger2 = false;
  
  // Timing variables, used to monitor recording times
  unsigned long lastConnectionTime = 0;
  unsigned long sampletime_ms = 20000;          // Time to record one set of data - 20 seconds
  unsigned long recording_interval = 90000;    // Time to stay awake to record multiple sets of data before going to sleep
  static unsigned long start_time;
  
  
  // Debugging LED to test Sleep Mode
  int test_LED = 13;   // External LED useful but not necessary, between pin 13 and GND
  int test_LED2 = A0;  // External LED between Analog pin 0 and GND allows LED's without breadboards or resistors
  
  
  void setup()
  {    
    // Initialize serial communications and wait for port to open:
    Serial.begin(9600);
    //Serial.begin(115200);
    Serial.println("Starting to Setup..."); // F-Macro saves Ram space when writing to Serial Monitor
  
    pinMode(channel_1, INPUT);  // Coarse Particle Sensor Pin
    pinMode(channel_2, INPUT);  // Fine Particle Sensor Pin
    pinMode(test_LED, OUTPUT);
    pinMode(test_LED2, OUTPUT);
    
    /*__________SLEEP SETUP REQUIREMENTS______(Tony Dicola)______*/
    
    // Setup the watchdog timer to run an interrupt which wakes the Arduino from sleep every 8 seconds.
    // Note that the default behavior of resetting the Arduino with the watchdog will be disabled.
    
    // This next section of code is timing critical, so interrupts are disabled.
    // See more details of how to change the watchdog in the ATmega328P datasheet around page 50, Watchdog Timer.
    noInterrupts();
    // Set the watchdog reset bit in the MCU status register to 0.
    MCUSR &= ~(1<<WDRF);
    // Set WDCE and WDE bits in the watchdog control register.
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set watchdog clock prescaler bits to a value of 8 seconds.
    WDTCSR = (1<<WDP0) | (1<<WDP3);
    // Enable watchdog as interrupt only (no reset).
    WDTCSR |= (1<<WDIE);
    // Enable interrupts again.
    interrupts();
    /*__________END SLEEP SETUP REQUIREMENTS____________*/
    
    // connection state
    boolean notConnected = true;
  
    // Start GSM shield - Don't proceed until a connection is established
    // If your SIM has PIN, pass it as a parameter of begin() in quotes
    while(notConnected)
    {
      if(gsmAccess.begin(PINNUMBER)==GSM_READY &
        (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)==GPRS_READY))
        notConnected = false;
      else
      {
        //Serial.println("Not connected"); UNCOMMENT
        delay(1000);
      }
    }
  
    Serial.println("");
    Serial.println("GSM initialized"); 
    Serial.println("Setup complete \n");
  }
  
  void loop()
  {
    // Don't do anything unless the watchdog timer interrupt has fired.
    // Watchdog timer should fire every 8 seconds
    if (watchdogActivated){
          
          for (int x = 0; x < 3; x++)
          {
          digitalWrite(test_LED, HIGH);    // Light blinks when Watch Dog Timer bites (Debugging)
          delay(100);
          digitalWrite(test_LED, LOW);
          delay(100);
          }
          
          // Reset Watch Dog Timer (WDT)
          watchdogActivated = false;
          
          Serial.print("watch "); 
          Serial.print("dog "); 
          Serial.print("bite #"); 
          Serial.println(sleepIterations + 1);   
          
          // Increase the count of sleep iterations and take a sensor
          // reading once the max number of iterations has been hit.
          sleepIterations += 1;
  
          if (sleepIterations >= MAX_SLEEP_ITERATIONS) { // If the Arduino has slept through enough WDT cycles...
          
                  // Reset the number of sleep iterations.
                  sleepIterations = 0; 
                  digitalWrite(test_LED, HIGH);    // Arduino is Awake when the light is on
                  start_time = millis();
                  lastConnectionTime = millis();
                  
                  while(millis() - start_time <= recording_interval) // Record multiple data sets before going back to sleep
                  {                
                        if (!client.connected())  // As long as you're not sending anything, proceed
                              {
                              digitalWrite(test_LED, HIGH);
                              
                              valueP1 = digitalRead(channel_1);
                              valueP2 = digitalRead(channel_2);
                              
                              // Can be used, but is less accurate than manual counting
                              //pulseLength 1 or 2 = pulseIn(pin, LOW);        
                              //totalDuration 1 or 2 = totalDuration 1 or 2 + pulseLength 1/2;
                              
                              // Channel 1 Record Time when a particle is seen (1-10 micron particles)
                              if(valueP1 == LOW && trigger1 == false){
                                trigger1 = true;
                                triggerOn1 = micros();
                              }
                              
                              // Record Time when particles go away
                              if(valueP1 == HIGH && trigger1 == true){
                                triggerOff1 = micros();
                                pulseLength1 = triggerOff1 - triggerOn1;
                                totalDuration1 = totalDuration1 + pulseLength1;
                                trigger1 = false;
                              }
                              
                              // Channel 2 Record Time (2.5-10 micron particles)
                              if(valueP2 == LOW && trigger2 == false){
                                trigger2 = true;
                                triggerOn2 = micros();
                              }
                              
                              if(valueP2 == HIGH && trigger2 == true){
                                triggerOff2 = micros();
                                pulseLength2 = triggerOff2 - triggerOn2;
                                totalDuration2 = totalDuration2 + pulseLength2;
                                trigger2 = false;
                              }
                              
                              }
                                
                        if (client.available())
                              {
                                char c = client.read();
                                //Serial.print(c);        // For debuggin purposes only, to confirm connection with server
                              }
                        
                        // if you're not receiving info from Dweet, and enough seconds have passed since
                        // your last connection, then connect again and send data:
                        if(!client.connected() && ((millis() - lastConnectionTime) > sampletime_ms))
                              {
                                //Serial.print("");
                                //Serial.print( "Sample Duration 1: "); Serial.println(totalDuration1);
                                //Serial.print( "Sample Duration 2: "); Serial.println(totalDuration2);
                                sendData(totalDuration1, totalDuration2);
                              }
                                
                          // store the state of the connection for next time through the loop
                          lastConnected = client.connected(); 
                   }
               
          Serial.println("All data sent.");
          //client.stop();    // Break the GSM connection to prepare for sleep mode
          
          }  
          
    }   
    
    Serial.println( "going to sleep \n"); 
    digitalWrite(test_LED, LOW);   // LED off when Arduino Sleeping
    sleep();
  } 
  
  
  void sendData(unsigned long sampleDuration1, unsigned long sampleDuration2)
  {   
    // if there's a successful connection:
    if (client.connect(server, 80))
    {
      analogWrite(test_LED2, 200); // Second LED on when Data being uploaded to web
      Serial.println( "\nPreparing to send data... \n");
      
      // Get character length of incoming data
      String dataLength = String(sampleDuration1);
      int dataLengthReal = dataLength.length();
  
      // Calculate the ratio of time that particles are being counted on average
      ratio1 = sampleDuration1 / (sampletime_ms*10.0); // Divide micros() by 1000 to equate with milliseconds, then multiply by 100 to get percentage
      ratio2 = sampleDuration2 / (sampletime_ms*10.0); // Interger percentage 0=>100
      
      //Serial.print("Sample Duration 1: "); Serial.println(sampleDuration1); 
      //Serial.print("Ratio 1: "); Serial.println(ratio1); 
      //Serial.print("Sample Duration 2: "); Serial.println(sampleDuration2); 
      //Serial.print("Ratio 2: "); Serial.println(ratio2); 
      
      // Calculate the concentration per 0.01 cubic foot (Chris Nafis extrapolated from Shinyei data sheet)
      // http://www.howmuchsnow.com/arduino/airquality/grovedust/
      float particleCount1 = 1.1*pow(ratio1,3)-3.8*pow(ratio1,2)+520*ratio1+0.62; // # of particles 1-10 micrometers across per 0.01 cubic ft
      float particleCount2 = 1.1*pow(ratio2,3)-3.8*pow(ratio2,2)+520*ratio2+0.62; // # of particles 2.5-10 micrometers across per 0.01 cubic ft
      
      float coarse_particle_count = particleCount2;                   // Only large particles
      float fine_particle_count = particleCount1 - particleCount2;    // Only small particles
      
      int fineInt = int(fine_particle_count);
      int coarseInt = int(coarse_particle_count);
      
      // Assumed values of calculation constants (Courtesy of: Matthew Schroyer)
      // http://www.mentalmunition.com/2013/10/measure-air-pollution-in-your-home-or.html
      float density = 1.65*pow(10,12);
      float K = 3531.5;
      float pi = 3.14159;      
       
      // Calculate large particle mass concentration (micrograms per cubic meter)
      float largeRadius = 2.6*pow(10,-6);
      float largeVolume = (4/3)*pi*pow(largeRadius,3);
      float largeMass = density*largeVolume;
      float largeMassConcentration = (coarse_particle_count)*K*largeMass;
      int coarseMassInt = int(largeMassConcentration);
      
      // Small Particle mass concentration (micrograms per cubic meter)
      float smallRadius = 0.44*pow(10,-6);
      float smallVolume = (4/3)*pi*pow(smallRadius,3);
      float smallMass = density*smallVolume;
      float smallMassConcentration = (fine_particle_count)*K*smallMass;
      int fineMassInt = int(smallMassConcentration); // Only relevant data (mass / volume) to determine dangerous air pollution
        
      // FOR VIEWING IN SERIAL MONITOR
      Serial.print("");
      Serial.print("Large Concentration: "); Serial.println(coarse_particle_count); 
      Serial.print("Large Mass Concentration: "); Serial.println(largeMassConcentration); 
      Serial.print("");
      Serial.print("Small Concentration: "); Serial.println(fine_particle_count);
      Serial.print("Small Mass Concentration: "); Serial.println(smallMassConcentration);
      Serial.print("\n\n");
      
      // send the HTTP PUT request:
      client.print("GET /dweet/for/");
      client.print(THINGNAME);
      client.print("?FineParticleMassConcentration=");  // Don't use spaces for data names
      client.print(fineMassInt);
      client.print("&FineParticleCount=");
      client.print(fineInt);
      client.print("&CoarseParticleMassConcentration=");
      client.print(coarseMassInt);
      client.print("&CoarseParticleCount=");
      client.print(coarseInt);
      client.println(" HTTP/1.1");
      client.println("Host: dweet.io");
      client.println("Connection: close");
      client.println();
     
      } 
      
      else
      {
        // if you couldn't make a connection:
        Serial.println( "Problem when sending data...\n"); 
        //client.stop();
      }
      
      // reset the count
      totalDuration1 = 0;
      totalDuration2 = 0;
      
      // note the time that the connection was made or attempted
      lastConnectionTime = millis();
      analogWrite(test_LED2, 0);
  }
  
  
  /*____________SLEEP MODE FUNCTIONS____________*/ 
  
  // Define watchdog timer interrupt.
  ISR(WDT_vect)
  {
    // Set the watchdog activated flag.
    // Note that you shouldn't do much work inside an interrupt handler.
    watchdogActivated = true;
  }
  
  // Put the Arduino to sleep.
  void sleep()
  {
    // Set sleep to full power down.  Only external interrupts or 
    // the watchdog timer can wake the CPU!
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
    // Turn off the ADC while asleep.
    power_adc_disable();
  
    // Enable sleep and enter sleep mode.
    sleep_mode();
  
    // CPU is now asleep and program execution completely halts!
    // Once awake, execution will resume at this point.
    
    // When awake, disable sleep mode and turn on all devices.
    sleep_disable();
    power_all_enable();
  }
  
  
  /*____________END SLEEP MODE FUNCTIONS____________*/ 
  
  
  
  
  

