

class ClientLogic():
    def __init__(self):
        pass
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
                    order = None
                case "Canceled":
                    order = None
            return order    
        except:
            return order
