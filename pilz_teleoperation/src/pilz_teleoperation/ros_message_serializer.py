import genpy
import rospy

class RosMessageSerializer:
    """
    Serialization and De-Serialization of ROS messages for convenient file printing
    """
    def __init__(self):
        self._module_imports = {}

    def write_messages_to_file(self, ros_messages, filename):
        """
        Writes a file that can be used with 'import' to restore
        :param ros_messages: iterable of messages to be serialized.
                tuple contains the variable name used for serialization and a ROS message object
        :param filename: target filename, overwritten if existent
        """
        with open(filename, 'w') as f:
            serialized_message = ""
            for varname, msg in ros_messages:
                serialized_message += "{} = {}\n".format(varname, self.convert_ros_message_to_python(msg))
            f.write("\n".join("from {} import {}".format(module, typename) for typename, module in self._module_imports.items()))
            f.write("\n\n")
            f.write(serialized_message)

    def convert_ros_message_to_python(self, message, indentation=0):
        """
        Serialize a ROS msg into a String Object that can be used with eval() afterwards
        :rtype: String
        """
        if not isinstance(message, (genpy.Message, rospy.rostime.Time, rospy.rostime.Duration)):
            # end of recursion
            if isinstance(message, str):
                return "'{}'".format(message)
            else:
                return str(message)

        field_values = []  # ordered list with string-converted attribute values
        for field_name, field_type in self._get_message_fields(message):
            val = self.convert_ros_message_to_python(getattr(message, field_name), indentation + 2)
            field_values.append((field_name,val))
        
        result = ',\n'.join(" "*(indentation+2) + '{} = {}'.format(key,value) for key, value in field_values)
        if type(message).__name__ not in self._module_imports.keys():
            self._module_imports[type(message).__name__] = type(message).__module__

        return "{}(\n{}\n{})".format(type(message).__name__, result, " " * indentation)

    def _get_message_fields(self, message):
        if isinstance(message, (rospy.rostime.Time, rospy.rostime.Duration)):
            return [('secs', int), ('nsecs', int)]
        else:
            return zip(message.__slots__, message._slot_types)
