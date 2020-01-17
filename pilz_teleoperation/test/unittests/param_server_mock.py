# Copyright (c) 2020 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import yaml
import os

_package_dir = "/".join(os.path.dirname(os.path.realpath(__file__)).split("/")[:-2])
with open(_package_dir + "/config/teleoperation_settings.yaml") as f:
    setting_defaults = yaml.load(f.read())


def mocked_get_param(key, *args, **kwargs):
    return setting_defaults[key[1:]]
