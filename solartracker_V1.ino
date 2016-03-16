/**************************** 
  Mobile Sun-Tracking Solar Power Plant
  Arduino Microcontroller
  
  
  ****************************

  
  Rev 1.2 

*****************************/
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

//#define DEBUG                                        // Uncomment to enable debug messages, sent to serial port; set DEBUG_LEVEL to desired setting
#define DEBUG_LEVEL_LOW         1                      // Minimum number of debug log messages sent out
#define DEBUG_LEVEL_MEDIUM      2
#define DEBUG_LEVEL_HIGH        3                      // Maximum number of debug log messages sent out
#define DEBUG_LEVEL             DEBUG_LEVEL_LOW        // Set to desired level of debug logging

#define SERVO_START_POSITION   90                      // Solar panel should be faced south at 90; then 60 is west and 120 is east
#define MOTOR_SPEED           180                      // Speed of the linear actuator (elevation) 
#define MOTOR_TIMEOUT        6000L                     // Timeout in msec when moving the linear actuator to a new position

// Digital pin assignments
#define SERVO_PWM_PIN           9                      // Servo pulse-width-modulation pin

// Analog pin assignments
#define PHOTOCELL_Tl_PIN        0                      // A0, Photocell top left
#define PHOTOCELL_Tr_PIN        1                      // A1, Photocell top right
#define PHOTOCELL_Bc_PIN        2                      // A2, Photocell bottom center
#define MOTOR_POT_PIN           3                      // A3, Linear actuator position pot wiper

// Sensor limits
#define PHOTOCELL_RAW_MIN     100                      // Analog value when darkest
#define PHOTOCELL_RAW_MAX    1000                      // Analog value when brightest
#define MOTORPOS_RAW_MIN      520                      // Analog value of linear actuator wiper when solar panel is horizontal (full BACKWARD)
#define MOTORPOS_RAW_MAX      930                      // Analog value of linear actuator wiper when solar panel is vertical (full FORWARD)

#define LIGHT_DEADBAND_SERVO    2                      // Deadband for servo light sensor difference detection
#define LIGHT_DEADBAND_MOTOR    3                      // Deadband for linear actuator light sensor difference detection
#define SERVO_POS_CHANGE        3                      // Amount to change servo position each rotation update

#define DARKNESS                5                      // Minimum amount of light before the darkness of night
#define DARK_TIME           60000L                     // The number of msecs it must be before we say it's dark

#define AZIMUTH                 0                      // Axis is azimuth
#define ELEVATION               1                      // Axis is elevation

Adafruit_MotorShield MS = Adafruit_MotorShield( );     // Motor Shield object with the default I2C address
Adafruit_DCMotor *motorElevator = MS.getMotor( 4 );    // Linear actuator on motor port M4 elevates solar panel on horizontal axis
Servo servoRotator;                                    // Servo rotates solar panel on vertical axis

// Global Variables
int val_ServoPosition = 0;                             // Servo's current position; 60 - 90 - 120
int val_MotorPosition = 0;                             // Linear actuator's position; 0 to 100

int val_TopLeftLight = 0;                              // Scaled value from top left photocell 0 to 100
int val_TopRightLight = 0;                             // Scaled value from top right photocell 0 to 100
int val_BottomLight = 0;                               // Scaled value from bottom photocell; 0 to 100

int val_BottomTopDifference = 0;                       // Difference between top and bottom light sensors
int val_RightLeftDifference = 0;                       // Difference between left and right light sensors

int val_BrightestLight = 0;                            // Brightest light found during initialization
int val_BrightestServoPosition = 0;                    // Position of the servo at brightest light
int val_BrightestMotorPosition = 0;                    // Position of the linear actuator at brightest light

boolean val_IsDark = false;                            // Set to true if it's dark out
long val_DarkTimer = 0L;                               // Timer used to determine the darkness of night


void setup( )
{
  #ifdef DEBUG
  Serial.begin( 9600 );
  Serial.println( "Debug enabled..." );
  #endif
  
  DebugLog( DEBUG_LEVEL_LOW, "Debug level: ", DEBUG_LEVEL );

  MS.begin( );                                         // Start linear actuator with the default frequency 1.6KHz
  servoRotator.attach( SERVO_PWM_PIN );                // Attach servo to the PWM pin
    
  motorElevator->setSpeed( MOTOR_SPEED );              // Set the linear actuator speed from 0 (off) to 255 (max speed)
  motorElevator->run( RELEASE );                       // RELEASE removes power, stopping the linear actuator motor
  
  InitServoPosition( );                                // The servo rotates the solar panel; 60 (west) - 90 (south) - 120 (east)
  InitMotorPosition( );                                // The linear actuator controls the elevation of the solar panel.
                                                       // FORWARD is tilted down (pos limit 100); BACKWARD is tilted up (pos limit 0)
  val_DarkTimer = millis( );                           // Initialize the dark timer
}

// Continuously move solar panel towards brightest light.
// Check to see if it's night and if so, reposition the solar panel to face east.
//
void loop( ) 
{
  RotateSolarPanel( );
  ElevateSolarPanel( );
  FaceEastIfNight( );
}

//************************
// Initialize Servo Position
//   Sweep east to west and find the brightest light.
//   Update val_ServoPosition and rotate servo to that position.
//
void InitServoPosition( )
{
  DebugLog( DEBUG_LEVEL_LOW, "Searching for brightest AZIMUTH from ", SERVO_START_POSITION - 30 );

  GetServoAndMotorPosition( );
  
  // Face solar panel to far side then start sampling light as it moves to the other side
  servoRotator.write( SERVO_START_POSITION - 30 );
  delay( 4000 );
    
  int servoSearchPosition = -30;
  val_BrightestLight = 0;

  for( int i = 0; i < 12; i++ )
  {
    SampleForBrightestLight( AZIMUTH );          // Updates val_ServoPosition when brightest light found

    servoSearchPosition += 5;
    servoRotator.write( SERVO_START_POSITION + servoSearchPosition );
    delay( 500 );
  }
  
  DebugLog( DEBUG_LEVEL_LOW, "Brightest Servo Position: ", val_BrightestServoPosition );

  // Rotate servo to the position found to have the brightest light
  servoRotator.write( val_BrightestServoPosition );
  delay( 4000 );
  
  DebugLog( DEBUG_LEVEL_LOW, "Azimuth Initialized: ", servoRotator.read( ) );
}

//************************
// Initialize Motor Position
// Move solar panel from full FORWAD (vertical) to full BACKWARD (horizontal) while looking for the
// position with the brightest light, then move the solar panel so it's at that position.
//
void InitMotorPosition( )
{
  unsigned long startTime = 0;
  
  // Jog the linear actuator before moving it to the FORWARD limit
  motorElevator->run( BACKWARD );                         // Start moving linear actuator motor
  delay( 500 );
  motorElevator->run( RELEASE );                          // Stop the motor
  delay( 500 );
  motorElevator->run( FORWARD );                          // Start moving linear actuator motor
  delay( 500 );
  motorElevator->run( RELEASE );                          // Stop the motor

  GetServoAndMotorPosition( );

  // Move solar panel to full FORWARD position.
  // We could get stuck in these while loops if the linear actuator isn't moving for some reason (i.e. loss of 12v power to the motor).
  // Run while loop only for as long as the motor timeout period. If all goes well we should break out before that.
  motorElevator->run( FORWARD );
  startTime = millis( );
  
  while( val_MotorPosition < 100 )                        // Move linear actuator to full FORWARD limit
  {
    GetServoAndMotorPosition( );
    
    if( ( millis( ) - startTime ) > MOTOR_TIMEOUT )       // Break out if it looks like the motor isn't moving
    {
      DebugLog( DEBUG_LEVEL_LOW, "Motor move timeout!: ", 0 );
      break;
    } 
  }
  
  motorElevator->run( RELEASE );                          // Stop the motor
  delay( 2000 );                                          // Delay for 2 seconds; good time to remove power when preparing to store 
                                                          // or move the solar panel unit as it is now in the full FORWARD position
  
  DebugLog( DEBUG_LEVEL_LOW, "Searching for brightest ELEVATION from ", val_MotorPosition );
  
  // Now start moving solar panel towards full BACKWARD while looking for the position with the brightest light
  val_BrightestLight = 0;                                 // Clear brightest light value
  SampleForBrightestLight( ELEVATION );                   // Check for brightest light before moving the motor

  motorElevator->run( BACKWARD );                         // Start moving linear actuator motor
  startTime = millis( );
  
  while( val_MotorPosition > 0 )                          // Move linear actuator towards BACKWARD limit while looking for brightest light
  {
    GetServoAndMotorPosition( );
    SampleForBrightestLight( ELEVATION );                 // Updates val_BrightestMotorPosition when brightest light found
          
    if( ( millis( ) - startTime ) > MOTOR_TIMEOUT )       // Break out if it looks like the motor isn't moving
    {
      DebugLog( DEBUG_LEVEL_LOW, "Motor move timeout!: ", 1 );
      break;
    } 
  }
  
  DebugLog( DEBUG_LEVEL_LOW, "Brightest Motor Position: ", val_BrightestMotorPosition );
  
  // Move solar panel FORWARD to face the brightest light
  motorElevator->run( FORWARD );                          // Move linear actuator to position with brightest light
  startTime = millis( );
  
  while( val_MotorPosition < val_BrightestMotorPosition )
  {
    GetServoAndMotorPosition( );

    if( ( val_MotorPosition >= val_BrightestMotorPosition ) || ( val_MotorPosition >= 100 ) )  // Break out if we hit brightest light or FORWARD limit
      break;

    if( ( millis( ) - startTime ) > MOTOR_TIMEOUT )       // Break out if it looks like the motor isn't moving
    {
      DebugLog( DEBUG_LEVEL_LOW, "Motor move timeout!: ", 0 );
      break;
    }
  }

  motorElevator->run( RELEASE );                          // Stop the motor
  DebugLog( DEBUG_LEVEL_LOW, "Elevation Initialized: ", val_MotorPosition );
}

//************************
// Face East If Night
//   If all the light sensors are indicating darkness for a period of time, assume it's the darkness of night.
//   DARKNESS should be tuned to not detect bright moon light or über loud fire flies.
//   If it's dark then reposition the solar panel to face east and wait until sunrise.
//
void FaceEastIfNight( )
{
  GetLightSensorValues( );
  
  if( ( val_TopRightLight <= DARKNESS ) && ( val_TopLeftLight <= DARKNESS ) && ( val_BottomLight <= DARKNESS ) )
  {
    if( ( !val_IsDark ) && ( ( millis( ) - val_DarkTimer ) >= DARK_TIME ) )
    {
      // Still dark after timeout so it must be night
      // Face the solar panel low towards the east...
      
      servoRotator.write( SERVO_START_POSITION + 30 );
      delay( 5000 );
      
      motorElevator->run( FORWARD );
      delay( 5000 );
      motorElevator->run( RELEASE );
      
      // It's dark and the solar panel is facing east...
      // moon and stars, crickets, hoot owls, vampires... now we wait for sunrise.
      // I'm hearing Pink Floyd "Echos"
      
      val_IsDark = true;                    // Set dark flag to keep from trying to move the solar panel during the night
      DebugLog( DEBUG_LEVEL_LOW, "Night time: avg light - ", ( val_TopRightLight + val_TopLeftLight + val_BottomLight ) / 3 );
    }
  }
  else
  {
    if( val_IsDark )                        // If we were dark, we are no longer. Must be sunrise
      DebugLog( DEBUG_LEVEL_LOW, "Sunrise: avg light - ", ( val_TopRightLight + val_TopLeftLight + val_BottomLight ) / 3 );
      
    val_IsDark = false;                     // We have light so reset the dark flag to keep things moving
    val_DarkTimer = millis( );              // ...and reset the darkness timer
  }
}

//************************
// Sample For Brightest Light
//   Get the average of all the light sensors and keep the running brightest value in val_BrightestLight.
//   Save off the position of the servo and linear actuator in val_BrightestServoPosition  and val_BrightestMotorPosition.
//
void SampleForBrightestLight( int Axis )
{
  int lightAverage = 0;
  
  GetLightSensorValues( );
  lightAverage = ( val_TopRightLight + val_TopLeftLight + val_BottomLight ) / 3;
  
  if( lightAverage > val_BrightestLight )
  {
    val_BrightestLight = lightAverage;
    GetServoAndMotorPosition( );            // Get the current servo and motor positions
    
    if( Axis == AZIMUTH )
    {
      val_BrightestServoPosition = val_ServoPosition;
      DebugLog( DEBUG_LEVEL_MEDIUM, "SampleForBrightestLight( ) servo: ", val_BrightestServoPosition );
    }
    else   // ELEVATION
    {
      val_BrightestMotorPosition = val_MotorPosition;   
      DebugLog( DEBUG_LEVEL_MEDIUM, "SampleForBrightestLight( ) motor: ", val_BrightestMotorPosition );
    }
  }  
}

//************************
// Get Servo and Motor Position
//   Read in servo position and store in global val_ServoPosition.
//   Read linear actuator pot and scale from 0 to 100 where solar panel is horizontal at 0 (full BACKWARD)
//   and solar panel is vertical at 100 (full FORWARD). Store results in global val_MotorPosition.
//
void GetServoAndMotorPosition( )
{
  val_ServoPosition = servoRotator.read( );
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetServoAndMotorPosition( ) (servo): ", val_ServoPosition );
  
  val_MotorPosition = constrain( analogRead( MOTOR_POT_PIN  ), MOTORPOS_RAW_MIN, MOTORPOS_RAW_MAX );
  DebugLog( DEBUG_LEVEL_HIGH, "GetServoAndMotorPosition( ) (raw): ", val_MotorPosition );
  val_MotorPosition = map( val_MotorPosition, MOTORPOS_RAW_MIN, MOTORPOS_RAW_MAX, 0, 100 );
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetServoAndMotorPosition( ) (motor scaled): ", val_MotorPosition );
}

//************************
// Get Light Sensor Values
//   Read photocells and scale from 0 to 100 where 0 is darkest.
//   Store results in globals val_TopRightLight, val_TopLeftLight, and val_BottomLight.
//
void GetLightSensorValues( )
{
  val_TopRightLight = constrain( analogRead( PHOTOCELL_Tr_PIN ), PHOTOCELL_RAW_MIN, PHOTOCELL_RAW_MAX );
  DebugLog( DEBUG_LEVEL_HIGH, "GetLightSensorValues( ) top right (raw): ", val_TopRightLight );
  val_TopRightLight = map( val_TopRightLight, PHOTOCELL_RAW_MIN, PHOTOCELL_RAW_MAX, 0, 100 );
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetLightSensorValues( ) top right (scaled): ", val_TopRightLight );

  val_TopLeftLight = constrain( analogRead( PHOTOCELL_Tl_PIN ), PHOTOCELL_RAW_MIN, PHOTOCELL_RAW_MAX );  
  DebugLog( DEBUG_LEVEL_HIGH, "GetLightSensorValues( ) top left (raw): ", val_TopLeftLight );
  val_TopLeftLight = map( val_TopLeftLight, PHOTOCELL_RAW_MIN, PHOTOCELL_RAW_MAX, 0, 100 );  
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetLightSensorValues( ) top left (scaled): ", val_TopLeftLight );

  val_BottomLight = constrain( analogRead( PHOTOCELL_Bc_PIN ), PHOTOCELL_RAW_MIN, PHOTOCELL_RAW_MAX );  
  DebugLog( DEBUG_LEVEL_HIGH, "GetLightSensorValues( ) bottom center (raw): ", val_BottomLight );
  val_BottomLight = map( val_BottomLight, PHOTOCELL_RAW_MIN, PHOTOCELL_RAW_MAX, 0, 100 );  
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetLightSensorValues( ) bottom center (scaled): ", val_BottomLight );
    
  val_RightLeftDifference = val_TopRightLight - val_TopLeftLight;                // Difference between the right and left light sensors
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetLightSensorValues( ) right-left difference: ", val_RightLeftDifference );
  
  // Use the top light sensor getting the most light for calculating the bottom-top difference
  int topLight = ( val_TopRightLight > val_TopLeftLight ) ? val_TopRightLight : val_TopLeftLight;
  val_BottomTopDifference = val_BottomLight - topLight;                         // Difference between the bottom and top light sensors
  DebugLog( DEBUG_LEVEL_MEDIUM, "GetLightSensorValues( ) bottom-top difference: ", val_BottomTopDifference );
}

//************************
// Rotate towards brightest light
//
void RotateSolarPanel( )
{
  if( val_IsDark )
    return;                                            // If it's night, wait for sunrise

  int positionChange = 0;
  
  GetLightSensorValues( );
    
  if( abs( val_RightLeftDifference ) <= LIGHT_DEADBAND_SERVO )
    return;                                            // Facing brightest light, no need to rotate

  // Change servo position based on light sensor difference
  if( val_RightLeftDifference > 0 )
  {
    if( val_ServoPosition >= 120 )                     // Don't move servo if already at CW limit
      return;
      
    val_ServoPosition += SERVO_POS_CHANGE;             // Increment position for clockwise westward rotation
    DebugLog( DEBUG_LEVEL_LOW, "Rotate( ) CW: ", val_BottomTopDifference );
  }
  else
  {
    if( val_ServoPosition <= 60 )                      // Don't move servo if already at CCW limit
      return;

    val_ServoPosition -= SERVO_POS_CHANGE;             // Decrement position for counterclockwise eastward rotation
    DebugLog( DEBUG_LEVEL_LOW, "Rotate( ) CCW: ", val_BottomTopDifference );
  }

  servoRotator.write( val_ServoPosition );             // Rotate solar panel towards brightness
  delay( 100 );
}

//************************
// Elevate towards brightest light
//
void ElevateSolarPanel( )
{
  if( val_IsDark )
    return;                                           // If it's night, wait for sunrise

  GetLightSensorValues( );
  GetServoAndMotorPosition( );

  if( abs( val_BottomTopDifference ) <= LIGHT_DEADBAND_MOTOR )
    return;                                           // Facing brightest light so just return
  
  // Change linear actuator position based on light sensor difference
  if( val_BottomTopDifference > 0 )
  {
    if( val_MotorPosition >= 100 )                   // Don't start motor if already at forward limit
      return;
      
    motorElevator->run( FORWARD );                   // Elevate solar panel towards brightness
    DebugLog( DEBUG_LEVEL_LOW, "Elevate( ) FORWARD: ", val_BottomTopDifference );
  }
  else
  {
    if( val_MotorPosition <= 0 )                     // Don't start motor if already at backward limit
      return;
      
    motorElevator->run( BACKWARD );                  // Elevate solar panel towards brightness
    DebugLog( DEBUG_LEVEL_LOW, "Elevate( ) BACKWARD: ", val_BottomTopDifference );
  }
  
  delay( 100 );                                      // Short delay to move a little bit at a time...
  motorElevator->run( RELEASE );                     // Ok that's enough, stop the linear actuator
}

void DebugLog( int Level, char *Label, int Var )
{
  #ifdef DEBUG
  if( Level <= DEBUG_LEVEL )
  {
    Serial.print( Label );
    Serial.println( Var );
  }
  #endif
}
