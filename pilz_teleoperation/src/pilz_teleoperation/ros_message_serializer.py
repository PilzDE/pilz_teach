import genpy
import rospy
from collections import OrderedDict


class RosMessageSerializer(object):
    """
    Serialization and De-Serialization of ROS messages for convenient file printing
    """

    def __init__(self):
        self._module_imports = {}

    def write_messages_to_file(self, ros_messages, filename):
        """
        Writes a file that can be used with 'import' to restore
        :param ros_messages: iterable [ordered] dictionary of messages to be serialized.
                key is the variable name used for serialization and value a ROS message object
        :param filename: target filename, overwritten if existent
        """
        with open(filename, 'w') as f:
            serialized_message = ""
            for varname, msg in ros_messages.items():
                serialized_message += "{} = {}\n".format(varname, self.convert_ros_message_to_python(msg))
            f.write("\n".join(
                "from {} import {}".format(module, typename) for typename, module in self._module_imports.items()))
            f.write("\n\n")
            f.write(serialized_message)
            f.write("\n")

    def convert_ros_message_to_python(self, message, indentation=0):
        """
        Serialize a ROS msg into a String Object that can be used with eval() afterwards
        :rtype: String
        """
        if not isinstance(message, (genpy.Message, rospy.rostime.Time, rospy.rostime.Duration)):
            # end of recursion
            return self._message_to_string(message)

        field_values = self._message_to_stringdict(message, indentation)
        result = ',\n'.join(
            '{}{} = {}'.format(" " * (indentation + 2), key, value) for key, value in field_values.items())
        self._module_imports[type(message).__name__] = type(message).__module__

        return "{}(\n{}\n{})".format(type(message).__name__, result, " " * indentation)

    def _message_to_stringdict(self, message, indentation):
        field_values = OrderedDict()
        for field_name in self._get_message_fields(message):
            # recursion:
            field_values[field_name] = self.convert_ros_message_to_python(getattr(message, field_name), indentation + 2)
        return field_values

    @staticmethod
    def _get_message_fields(message):
        if isinstance(message, (rospy.rostime.Time, rospy.rostime.Duration)):
            return ['secs', 'nsecs']
        else:
            return message.__slots__

    @staticmethod
    def _message_to_string(message):
        if isinstance(message, str):
            return "'{}'".format(message)
        else:
            return str(message)
