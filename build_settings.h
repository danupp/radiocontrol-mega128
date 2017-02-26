
// ** Select architecture of operation. **
 
// Direct frequency - DAC A channel tunes directly to the dial frequency + IF. This is the mode used with the simple radio board.
// Indirect frequency - DAC A generates a reference frequency to be used in a PLL tuning to the dial frequency +/- IF

#define DIRECT_FREQ 
//#define INDIRECT_FREQ


// ** Intermediate frequency (IF) in kHz **

#define F_IF 45000


// ** Clarifier **

// Comment out if no clarifier-potentiometer is connected. The clarifier adds fine tuning of the frequency in receive mode. 

//#define CLARPOT


// ** Bands **
// HF_HAM - Shortwave. Band button jumps between the amateur bands: 1.8, 3.5, 7, 10, 14, 18, 21, 24, 28 MHz
// HF_BC - Shortwave. Band button jumps between the broadcast bands: 
// TWOMETER - The 2m / 144 MHz amateur band.
// TWOMETER_TRANSV - The 144 MHz amateur band with transverter to microwave bands

#define  HF_HAM
//#define HF_BC
//#define TWOMETER
//#define TWOMETER_TRANSV
