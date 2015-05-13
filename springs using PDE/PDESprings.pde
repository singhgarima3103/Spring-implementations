float Stiffness = 5.0;
float BobMass = 0.5;
int StateSize = 3;
float[] InitStateFE = new float[StateSize];
float[] StateFE = new float[StateSize];

float[] InitStateEC = new float[StateSize];
float[] StateEC = new float[StateSize];

float[] InitStateRK4 = new float[StateSize];
float[] StateRK4 = new float[StateSize];

int StateCurrentTime = 0;
int StatePositionX = 1;
int StateVelocityX = 2;

int WindowWidthHeight = 600;
float WorldSize = 2.0;
float PixelsPerMeter;
float OriginPixelsX;
float OriginPixelsY;

void setup()
{
    // Create initial state. FE
    InitStateFE[StateCurrentTime] = 0.0;
    InitStateFE[StatePositionX] = 0.65;
    InitStateFE[StateVelocityX] = 0.0;

    // Copy initial state to current state.
    // notice that this does not need to know what the meaning of the
    // state elements is, and would work regardless of the state's size.
    for ( int i = 0; i < StateSize; ++i )
    {
        StateFE[i] = InitStateFE[i];
    }
    
    
    
    // Create initial state. EC
    InitStateEC[StateCurrentTime] = 0.0;
    InitStateEC[StatePositionX] = 0.65;
    InitStateEC[StateVelocityX] = 0.0;

    // Copy initial state to current state.
    // notice that this does not need to know what the meaning of the
    // state elements is, and would work regardless of the state's size.
    for ( int i = 0; i < StateSize; ++i )
    {
        StateEC[i] = InitStateEC[i];
    }


    // Create initial state. RK4
    InitStateRK4[StateCurrentTime] = 0.0;
    InitStateRK4[StatePositionX] = 0.65;
    InitStateRK4[StateVelocityX] = 0.0;

    // Copy initial state to current state.
    // notice that this does not need to know what the meaning of the
    // state elements is, and would work regardless of the state's size.
    for ( int i = 0; i < StateSize; ++i )
    {
        StateRK4[i] = InitStateRK4[i];
    }


    // Set up normalized colors.
    colorMode( RGB, 1.0 );
    
    // Set up the stroke color and width.
    stroke( 0.0 );
    //strokeWeight( 0.01 );
    
    // Create the window size, set up the transformation variables.
    size( WindowWidthHeight, WindowWidthHeight );
    PixelsPerMeter = (( float )WindowWidthHeight ) / WorldSize;
    OriginPixelsX = 0.5 * ( float )WindowWidthHeight;
    OriginPixelsY = 0.5 * ( float )WindowWidthHeight;
    textSize( 24 );
}

// Draw our State, with the unfortunate units conversion.
void DrawStateFE()
{
    // Compute end of arm.
    float SpringEndX = PixelsPerMeter * StateFE[StatePositionX];

    // Compute the CORRECT position.
    float sqrtKoverM = sqrt( Stiffness / BobMass );
    float x0 = InitStateFE[StatePositionX];
    float v0 = InitStateFE[StateVelocityX];
    float t = StateFE[StateCurrentTime];
    float CorrectPositionX = ( x0 * cos( sqrtKoverM * t ) ) +
        ( ( v0 / sqrtKoverM ) * sin( sqrtKoverM + t ) );
    
    // Compute draw pos for "correct"
    float CorrectEndX = PixelsPerMeter * CorrectPositionX;



    // Draw the spring.
    strokeWeight( 1.0 );
    line( 0.0, -150.0, SpringEndX, -150.0 );
          
    // Draw the spring pivot
    fill( 0.0 );
    ellipse( 0.0, -150.0, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the spring bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( SpringEndX, -150.0, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );
             
             
             
    // Draw the correct bob in blue
    fill( 0.0, 0.0, 1.0 );
    ellipse( CorrectEndX, -PixelsPerMeter * 0.25,
             PixelsPerMeter * 0.1,
             PixelsPerMeter * 0.1 );
}



// Draw our State, with the unfortunate units conversion.
void DrawStateEC()
{
    // Compute end of arm.
    float SpringEndX = PixelsPerMeter * StateEC[StatePositionX];


    // Draw the spring.
    strokeWeight( 1.0 );
    line( 0.0, 80.0, SpringEndX, 80.0 );
          
    // Draw the spring pivot
    fill( 0.0 );
    ellipse( 0.0, 80.0, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the spring bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( SpringEndX, 80.0, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );

}


// Draw our State, with the unfortunate units conversion.
void DrawStateRK4()
{
    // Compute end of arm.
    float SpringEndX = PixelsPerMeter * StateRK4[StatePositionX];


    // Draw the spring.
    strokeWeight( 1.0 );
    line( 0.0, 250.0, SpringEndX, 250.0 );
          
    // Draw the spring pivot
    fill( 0.0 );
    ellipse( 0.0, 250.0, 
             PixelsPerMeter * 0.03, 
             PixelsPerMeter * 0.03 );
    
    // Draw the spring bob
    fill( 1.0, 0.0, 0.0 );
    ellipse( SpringEndX, 250.0, 
             PixelsPerMeter * 0.1, 
             PixelsPerMeter * 0.1 );

   
}

// Time Step function.
void TimeStepFE( float i_dt )
{
    // Compute acceleration from current position.
    float A = ( -Stiffness / BobMass ) * StateFE[StatePositionX];

    // Update position based on current velocity.
    StateFE[StatePositionX] += i_dt * StateFE[StateVelocityX];

    // Update velocity based on acceleration.
    StateFE[StateVelocityX] += i_dt * A;

    // Update current time.
    StateFE[StateCurrentTime] += i_dt;
}


// Euler-Cromer Time Step function.
void TimeStepEC( float i_dt )
{
    // Compute acceleration from current position.
    float A = ( -Stiffness / BobMass ) * StateEC[StatePositionX];
    
    // Update velocity based on acceleration.
    StateEC[StateVelocityX] += i_dt * A;

    // Update position based on current velocity.
    StateEC[StatePositionX] += i_dt * StateEC[StateVelocityX];

    // Update current time.
    StateEC[StateCurrentTime] += i_dt;
}

// Acceleration from Position.
float A_from_X( float i_x )
{
    return -( Stiffness / BobMass ) * i_x;
}

// Time Step function.
void TimeStepRK4( float i_dt )
{
    float vStar1 = StateRK4[StateVelocityX];
    float aStar1 = A_from_X( StateRK4[StatePositionX] );

    float vStar2 = StateRK4[StateVelocityX] + ( ( i_dt / 2.0 ) * aStar1 );
    float xTmp2 = StateRK4[StatePositionX] + ( ( i_dt / 2.0 ) * vStar1 );
    float aStar2 = A_from_X( xTmp2 );

    float vStar3 = StateRK4[StateVelocityX] + ( ( i_dt / 2.0 ) * aStar2 );
    float xTmp3 = StateRK4[StatePositionX] + ( ( i_dt / 2.0 ) * vStar2 );
    float aStar3 = A_from_X( xTmp3 );

    float vStar4 = StateRK4[StateVelocityX] + ( i_dt * aStar3 );
    float xTmp4 = StateRK4[StatePositionX] + ( i_dt * vStar3 );
    float aStar4 = A_from_X( xTmp4 );

    StateRK4[StatePositionX] += ( i_dt / 6.0 ) * 
        ( vStar1 + (2.0*vStar2) + (2.0*vStar3) + vStar4 );
    StateRK4[StateVelocityX] += ( i_dt / 6.0 ) * 
        ( aStar1 + (2.0*aStar2) + (2.0*aStar3) + aStar4 );

    // Update current time.
    StateRK4[StateCurrentTime] += i_dt;
}


// Processing Draw function, called every time the screen refreshes.
void draw()
{
    // Time Step.
    TimeStepFE( 1.0/50.0 );
    
   // Time Step.
    TimeStepEC( 1.0/50.0 );
    
    // Time Step.
    TimeStepRK4( 1.0/50.0 );

    // Clear the display to a constant color.
    background( 0.75 );
    
     // Label.
    fill( 1.0 );
    text( "forward euler", 10, 30 );
    
     // Label.
    fill( 1.0 );
    text( "Euler-Cromer", 10, 260 );
    
     // Label.
    fill( 1.0 );
    text( "RK4", 10, 480 );

    // Translate to the origin.
    translate( OriginPixelsX, OriginPixelsY );

    // Draw the simulation
    DrawStateFE();
    
    // Draw the simulation
    DrawStateEC();
    
    // Draw the simulation
    DrawStateRK4();
}


// Reset function. If the key 'r' is released in the display, 
// copy the initial state to the state.
void keyReleased()
{
    if ( key == 114 )
    {
        for ( int i = 0; i < StateSize; ++i )
        {
            StateFE[i] = InitStateFE[i];
            StateEC[i] = InitStateEC[i];
            StateRK4[i] = InitStateRK4[i];
        }
    }  
}
