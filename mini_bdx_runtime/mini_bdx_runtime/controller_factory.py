from mini_bdx_runtime.xbox_controller import XBoxController
from mini_bdx_runtime.ps5_controller import PS5Controller


class ControllerFactory:
    """Factory class to create the appropriate controller based on type"""
    
    CONTROLLER_TYPES = {
        'xbox': XBoxController,
        'ps5': PS5Controller,
        'playstation5': PS5Controller,
        'dualsense': PS5Controller,
    }
    
    @staticmethod
    def create_controller(controller_type: str, command_freq: int, only_head_control: bool = False):
        """
        Create a controller instance based on the specified type.
        
        Args:
            controller_type (str): Type of controller ('xbox', 'ps5', 'playstation5', 'dualsense')
            command_freq (int): Command frequency in Hz
            only_head_control (bool): Whether to enable only head control mode
            
        Returns:
            Controller instance (XBoxController or PS5Controller)
            
        Raises:
            ValueError: If controller type is not supported
        """
        controller_type = controller_type.lower().strip()
        
        if controller_type not in ControllerFactory.CONTROLLER_TYPES:
            available_types = ', '.join(ControllerFactory.CONTROLLER_TYPES.keys())
            raise ValueError(f"Unsupported controller type: {controller_type}. "
                           f"Available types: {available_types}")
        
        controller_class = ControllerFactory.CONTROLLER_TYPES[controller_type]
        
        try:
            return controller_class(command_freq, only_head_control)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize {controller_type} controller: {e}")
    
    @staticmethod
    def get_available_controllers():
        """Get list of available controller types"""
        return list(ControllerFactory.CONTROLLER_TYPES.keys())
    
    @staticmethod
    def auto_detect_controller():
        """
        Auto-detect connected controller type based on device name.
        Returns the detected controller type or 'xbox' as default.
        """
        import pygame
        pygame.init()
        
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected")
        
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        controller_name = joystick.get_name().lower()
        
        # Common PS5 controller identifiers
        ps5_identifiers = ['dualsense', 'ps5', 'playstation 5', 'sony interactive']
        
        # Common Xbox controller identifiers  
        xbox_identifiers = ['xbox', 'microsoft', 'x-input']
        
        if any(identifier in controller_name for identifier in ps5_identifiers):
            return 'ps5'
        elif any(identifier in controller_name for identifier in xbox_identifiers):
            return 'xbox'
        else:
            print(f"Unknown controller: {controller_name}. Defaulting to Xbox mapping.")
            return 'xbox'


if __name__ == "__main__":
    # Test the factory
    try:
        detected_type = ControllerFactory.auto_detect_controller()
        print(f"Detected controller type: {detected_type}")
        
        controller = ControllerFactory.create_controller(detected_type, 20)
        print(f"Successfully created {detected_type} controller")
        
        # Test for a few seconds
        import time
        for i in range(10):
            print(controller.get_last_command())
            time.sleep(0.5)
            
    except Exception as e:
        print(f"Error: {e}")