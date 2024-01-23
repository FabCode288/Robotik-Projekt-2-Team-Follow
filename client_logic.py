"""
Serves as an object in the clients main to process the last result and returns the next order
"""

class ClientLogic():
    def __init__(self):
        pass

    """
    Returns the next goal that the client should send depending on the the current value of result
    """
    def get_next_client_order(self,result):
        try:
            match result:
                case "Aborted":
                    order = None
                case "Start":
                    order = "move"
                case "RFID_reached":
                    order = "turn"                
                case "Robot_detected":
                    order = "follow"
                case "Turn_succesfull":
                    order = "move"
                case "Follow_succesfull":
                    order = None #Would go into evasion if implemented
                case "Canceled":
                    order = None
            return order    
        except:
            return None
