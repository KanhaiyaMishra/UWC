# UnderWater Communication
This repository contains transceiver implementation code for two different modulation schemes
1. Pulse Position Modulation
2. OFDM Multicarrier Modulation

This code is specifically written for RedPitaya Board STEM-LAB 125-14 (http://redpitaya.readthedocs.io/en/latest/). For porting it to different hardware, one needs to change the ADC and DAC functions. Everything else will remain same.

For more details of how to import it to other hardware, read about continuous acquisition and streaming example codes in SINE_TX and SINE_RX folders. Similar methods can be implemented on other hardware.

## Getting Started


### Prerequisites
To use this code as it is Redpitaya STEMLAB-125-14 board is required. PC is needed to connect to and access the redpitaya board. For the board no specific driver installation is required (can be accesssed via SSH).

### Installing
On RedPitaya Board (STEM-LAB125-14), use SSH login to access the system.
Clone the master repo of redpitaya.
In the api folder, copy the new modified api files from new_api folder of this repo and replace.
Build the api ( locaiton of the build output is important, will be required while linking other programs).
Clone this repo as well in another directory. 
Edit output and link library locations in the Makefile .

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing
