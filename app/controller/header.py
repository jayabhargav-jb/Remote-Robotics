import random

def set_speed(right_speed, left_speed):
    """
    Function to set the speed of the motors
    
    @param:
        right_speed: int - Speed of the right motor 
        left_speed: int - Speed of the left motor

    @return:
        bool - Speed set success or failure

    right_speed & left_speed should be a number between -100 and 100,
    speed < 0 : moving backwards
    speed = 0 : stop
    speed > 0 : moving forward

    """

    if not (-100 <= right_speed <= 100 and -100 <= left_speed <= 100):
        return False
    
    # TODO: Code to set the motor speed goes here
    return True
    


def get_speed():
    """
    Function to get the speed of the motors
    
    @return:
        right_speed: int - Speed of the right motor 
        left_speed: int - Speed of the left motor

    right_speed & left_speed will be a number between -100 and 100,
    speed < 0 : moving backwards
    speed = 0 : stop
    speed > 0 : moving forward

    """

    # TODO: Code to reading the current motor speed
    return [random.randint(-100, 100), random.randint(-100, 100)]


def get_distance_to_obstacle():
    """
    Function to get the distance to the obstacle in front using the IR sensor
    
    @return:
        distance: float - Distance to the nearest obstacle in cm

    """

    # TODO: Code to getting the IR Sensor distance
    return random.randint(0, 100)

def get_color_grid():
    """
    Function to get the color values of the sensor array
    
    @return:
        color: [bool, bool, bool, bool, bool] - Ground color white or black

        white will be represented by a value of True, 
        and black will be represented by the value False

    """

    return [random.choice([True, False]) for _ in range(5)]

def start_encoder_pulse_counting():
    """
    Function to start the encoder pulse counting, on calling the function twice, will reset the counter

    Count reliable until 2,14,74,83,647 
    """

    try:

        # Reset the encoder counter & set the couting state to True
        # ds.reset_encoder_count()
        # ds.set_encoder_status(True)

        # Enable the interrupts on the encoder pin

        return True
    
    except Exception as e:

        # TODO: Throw exception to server over websocket
        return False


def stop_encoder_pulse_counting():
    """
    Function to stop the encoder pulse counting, will reset the counter value
    """

    try:

        # Reset the encoder counter & set the couting state to False
        ds.reset_encoder_count()
        ds.set_encoder_status(False)

        # Disable the interrupts on the encoder pin
        encoder = Pin(3, Pin.IN)
        encoder.irq(handler=None)

        return True
    
    except Exception as e:

        # TODO: Throw exception to server over websockets
        return False
    
def get_encoder_pulse_count():
    """
    Function to get the encoder pulse count, reliable count until 2,14,74,83,647
    
    @return:
        count: int - Encoder pulse count

    """

    return random.randint(0, 21467111)