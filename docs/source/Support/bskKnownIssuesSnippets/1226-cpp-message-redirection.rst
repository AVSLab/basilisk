- Since 2.12.0, assigning a C++ module output to a stand-alone message copied
  its initial contents instead of redirecting subsequent writes. This left
  shared circles messages in the OpNav scenarios at zero despite successful
  image processing. Python output assignment now redirects reads and writes
  again and retains the gateway for producers, readers, and recorders, including
  inputs connected through module registration methods. Native C++ copies and
  moves of redirected messages take independent snapshots without consuming the
  gateway payload.
- GitHub issue 1226 follow-up: Redirecting an output to another module's embedded
  output could retain only a non-owning proxy, allowing the source module to be
  deleted while its storage was still in use. Redirection now retains the source
  module, and readers and recorders retain that owner independently. Assigning a
  saved output proxy after its module has been deleted raises ``ReferenceError``.
