#!/usr/bin/env python3

# Copyright 2022-2024 Fraunhofer Italia Research

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.

# Holds the global variables used for implementing different backends


from enum import Enum
import threading
import wrapt

from .backends.ifcopenshell.ifcopenshell import IfcOpenShell

# More backends to come...


class BackendType(Enum):
    ## Enum for all the individual states

    IFC_OPEN_SHELL = 0
    # More backends to come...


global _backend_lock
_backend_lock = threading.RLock()  # Avoid race-condition with global recursive lock
global _backend  # Global variable for some BimInterface
_backend = None


@wrapt.synchronized(_backend_lock)
def switch_backend(backend_type: BackendType, bim_file: str) -> bool:
    ## Function for switching the backend to another one by changing the global variable
    #
    #  @param backend_type
    #    Type of the desired backend
    #  @param bim_file
    #    BIM file to be processed by the backend
    #  @return
    #    True if backend was switched successfully, else false

    if bim_file is not None:
        global _backend
        if backend_type is BackendType.IFC_OPEN_SHELL:
            _backend = IfcOpenShell(bim_file)
            return True
    return False
