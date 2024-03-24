"""Clearpath driver"""

import logging
import constants_1 
import serial, sys, os, time
import serial.tools.list_ports

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def find_teknic_com_port():
    teknic_com_port = None
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "VID:PID=2890:8022" in p.usb_info():
            teknic_com_port = p.device
    return teknic_com_port

class MotorsDriver:
    """Class to control the Clearpath and gripper motors.
    is
    
    Parameters
    ----------
    gear_ratios
        A list of floats containing the gear ratios for the motors.
    """
    def __init__(self, gear_ratios: list[float]) -> None:        
        self._is_running = False
       # self.gear_ratios = gear_ratios
        self._is_homed = False
        pass

    def connect(self) -> bool:
        """Connect to teknic and gripper serial ports"""
        teknic_com_port = find_teknic_com_port()
        if not teknic_com_port:
            sys.exit("Could not find teknic clearcore")
            logger.info("Failed connection to teknic")
            return 0

        self.teknic_port = serial.Serial(teknic_com_port, 9600, timeout=1)
        return 1
   
   
   
    def start(self) -> None:
        """Start the motors."""
        logger.info("In start")
        if not self._is_running:
            # Initialize the motors.
            self._is_running = True
        raise NotImplementedError()
    
    def stop(self) -> None:
        """Stop the motors."""
        logger.info("In stop")
        if self._is_running:
            # Stop the  motors.
            self._is_running = False
        raise NotImplementedError()
    
    def reset(self) -> None:
        """Reset the motors."""
        logger.info("In reset")
        self.stop()
        self.start()
    
    def home(self) -> bool:
        """Home the gantry motors."""
        logger.info("In homing clearpath motors")
        if self._is_homed
            logger.info("XYZ already homed, cannot home twice unless motors are re-enabled/power cycled")
            return 0

        self.teknic_port.write(bytearray("h\n", "ascii"))
        self._is_running = True
        while self._is_running:
            if self.teknic_port.read():
                self._is_running = False
        self._is_homed = True
        time.sleep(SLEEP_COMMAND)

        raise NotImplementedError()

    
    def is_homed(self) -> bool:
        """Check if the motors are homed.
        
        Returns
        -------
            True if the motors are homed.
            False, otherwise.
        """
        return self._is_homed
        raise NotImplementedError()
    

    def get_cfg(self) -> list[float]:
        """Get the current configuration.
        
        Returns
        -------
        A list of floats contianing the motor values.
        """
        logger.info("In get_cfg")
        raise NotImplementedError()
    
    def goto_XYZ(self, cfg: list[float], is_absolute: bool) -> None:
        """Go to a specific configuration.
        
        Parameters
        ----------
        cfg
            A list of floats containing the configuration values
            to go to.
        is_absolute
            If True, an absolute move is executed.
            If False, a relative move is executed.
        """
        logger.info(f"In goto_cfg with is_absolute {is_absolute}")

        raise NotImplementedError()
    
