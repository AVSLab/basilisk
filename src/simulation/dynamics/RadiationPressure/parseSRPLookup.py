#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

from xml.etree import ElementTree

import numpy as np


class SRPLookupTableHandler:
    """Class to handle an SRP Lookup table"""
    def __init__(self):
        self.sHatBLookup = np.zeros([1, 3])
        self.forceBLookup = np.zeros([1, 3])
        self.torqueBLookup = np.zeros([1, 3])

    def parseAndLoadXML(self, filePath):
        document = ElementTree.parse(filePath)

        sHatBTree = document.find('sHatBValues')
        forceBTree = document.find('forceBValues')
        torqueBTree = document.find('torqueBValues')

        self.sHatBLookup.resize([len(list(sHatBTree)), 3], refcheck=False)
        self.forceBLookup.resize([len(list(forceBTree)), 3], refcheck=False)
        self.torqueBLookup.resize([len(list(torqueBTree)), 3], refcheck=False)

        for node in list(sHatBTree):
            idx = int(node.attrib['index'])
            for value in list(node):
                if value.tag == 'value_1':
                    self.sHatBLookup[idx, 0] = value.text
                if value.tag == 'value_2':
                    self.sHatBLookup[idx, 1] = value.text
                if value.tag == 'value_3':
                    self.sHatBLookup[idx, 2] = value.text

        for node in list(forceBTree):
            idx = int(node.attrib['index'])
            for value in list(node):
                if value.tag == 'value_1':
                    self.forceBLookup[idx, 0] = value.text
                if value.tag == 'value_2':
                    self.forceBLookup[idx, 1] = value.text
                if value.tag == 'value_3':
                    self.forceBLookup[idx, 2] = value.text

        for node in list(torqueBTree):
            idx = int(node.attrib['index'])
            for value in list(node):
                if value.tag == 'value_1':
                    self.torqueBLookup[idx, 0] = value.text
                if value.tag == 'value_2':
                    self.torqueBLookup[idx, 1] = value.text
                if value.tag == 'value_3':
                    self.torqueBLookup[idx, 2] = value.text
