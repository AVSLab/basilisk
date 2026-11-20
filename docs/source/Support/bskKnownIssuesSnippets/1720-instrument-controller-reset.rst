- GitHub issue 1720: :ref:`simpleInstrumentController` did not write its output message during reset, so the
  previous image command stayed published until the first update of a new acquisition. Reset now writes a zero
  command and clears the hold state.
