
class Debouncing:
    def __init__(self, previous_input, current_input) -> None:
        self.previous_input = previous_input
        self.current_input = current_input
    
    def is_leading_edge(self, button_name):
        return (
            self.current_input[button_name] == 1
            and self.previous_input[button_name] == 0
        )

    def is_trailing_edge(self, button_name):
        return (
            self.current_input[button_name] == 0
            and self.previous_input[button_name] == 1
        )
