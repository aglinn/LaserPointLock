import ctypes


class DllWrapper:
    def __init__(self, dll_path):
        """
        Initializes the DllWrapper by loading the specified DLL.

        Args:
            dll_path (str): Path to the DLL file.
        """
        self.dll_path = dll_path
        self.dll = self._load_dll_functions()

    def _load_dll_functions(self):
        """
        Loads a DLL and returns references to its exported functions.

        Returns:
            ctypes.WinDLL: A reference to the loaded DLL.
        """
        return ctypes.WinDLL(self.dll_path)

    def run_dll_function(self, function_name, **kwargs):
        """
        Runs a function from the loaded DLL by its name, passing the specified kwargs.

        Args:
            function_name (str): Name of the function to run.
            **kwargs: Keyword arguments to pass to the function.

        Returns:
            Any: The result of the function call.
        """
        try:
            dll_function = getattr(self.dll, function_name)
            return dll_function(**kwargs)
        except AttributeError:
            raise ValueError(f"Function '{function_name}' not found in the DLL.")
        except Exception as e:
            raise RuntimeError(f"Failed to run the function '{function_name}': {e}")


class OmegaNetTc(DllWrapper):

    def __int__(self, path_to_dll):
        # init the DLL wrapper class
        super().__init__(path_to_dll=path_to_dll)
        return

    def find_devices(self):
        

if __name__ == "__main__":
    # Replace 'example.dll' with the actual path to your DLL.
    dll_path = "example.dll"

    try:
        dll_wrapper = DllWrapper(dll_path)

        # Call the 'add' function in the DLL with arguments.
        result_add = dll_wrapper.run_dll_function("add", a=5, b=3)
        print("Result of add function:", result_add)

        # Call the 'subtract' function in the DLL with arguments.
        result_subtract = dll_wrapper.run_dll_function("subtract", a=10, b=4)
        print("Result of subtract function:", result_subtract)

        # Call any other function in the DLL with arguments.
        # Replace 'function_name_in_dll' with the actual function name in the DLL.
        result_custom_function = dll_wrapper.run_dll_function("function_name_in_dll", arg1=value1, arg2=value2)
        print("Result of custom function:", result_custom_function)

    except OSError as e:
        print(f"Failed to load the DLL: {e}")
