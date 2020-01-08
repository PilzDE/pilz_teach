from rospy_message_converter import message_converter
import genpy

class RosMessageSerializer:
    """
    Serialization and De-Serialization of ROS messages for convenient file printing
    """
    def __init__(self):
        self._module_imports = {}

    def write_messages_to_file(self, ros_messages, filename):
        """
        Writes a file that can be used with 'import' to restore
        :param ros_messages: dictionary of messages to be serialized.
                Key: contains the variable name used for serialization, Value a ROS Message Object
        :param filename: target filename, overwritten if existent
        """
        with open(filename, 'w') as f:
            serialized_message = ""
            for varname, msg in ros_messages.items():
                serialized_message += "{} = {}\n".format(varname, self.convert_ros_message_to_python(msg))
            f.write("\n".join("from {} import {}".format(module, typename) for typename, module in self._module_imports.items()))
            f.write("\n\n")
            f.write(serialized_message)

    def convert_ros_message_to_python(self, message, indentation=0):
        """
        Serialize a ROS msg into a String Object that can be used with eval() afterwards
        :rtype: String
        """
        if not isinstance(message, genpy.Message):
            return str(message)

        field_values = []  # ordered list with string-converted attribute values
        for field_name, field_type in message_converter._get_message_fields(message):
            val = self.convert_ros_message_to_python(getattr(message, field_name), indentation + 2)
            field_values.append((field_name,val))
        
        result = ',\n'.join(" "*(indentation+2) + '{} = {}'.format(key,value) for key, value in field_values)
        if type(message).__name__ not in self._module_imports.keys():
            self._module_imports[type(message).__name__] = type(message).__module__

        return "{}(\n{}\n{})".format(type(message).__name__, result, " " * indentation)
