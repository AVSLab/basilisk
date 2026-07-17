/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef MESSAGING_BASE_H
#define MESSAGING_BASE_H

/*! Holds type-erased pointers to a message's header and payload. */
struct messagePointerData
{
    void* header;  //!< pointer to the message header
    void* payload; //!< pointer to the message payload
};

/*! A base class for Message that enables type-erased pointer access. */
class MessageBase
{
  protected:
    messagePointerData pointers;  //!< stores the message's header/payload pointers
    messagePointerData reference; //!< cached storage for the borrowed pointer view

  public:
    /*!
     * Return a borrowed view of the type-erased message header and payload pointers.
     *
     * The returned ``messagePointerData`` structure is owned by this object and remains
     * valid only while the associated ``Message`` exists. Its header and payload addresses
     * are non-owning and remain valid only for the lifetime of that message.
     */
    messagePointerData* GetPointers(void)
    {
        reference.payload = pointers.payload;
        reference.header = pointers.header;
        return &reference;
    }
};

/*! A base class for ReadFunctor that enables type-erased pointer access. */
class ReadFunctorBase
{
  protected:
    void* headerVoidPtr;          //!< type-erased header pointer for external interface access
    void* payloadVoidPtr;         //!< type-erased payload pointer for external interface access
    messagePointerData reference; //!< cached storage for the borrowed pointer view

  public:
    //! constructor
    ReadFunctorBase()
      : headerVoidPtr(nullptr)
      , payloadVoidPtr(nullptr)
    {
    }

    /*!
     * Return a borrowed view of the type-erased source header and payload pointers.
     *
     * The returned ``messagePointerData`` structure is owned by this object and remains
     * valid only while the associated ``ReadFunctor`` exists. Its non-null header and
     * payload addresses are non-owning and remain valid only while the reader is subscribed
     * and the source message exists.
     */
    messagePointerData* GetPointers(void)
    {
        reference.header = headerVoidPtr;
        reference.payload = payloadVoidPtr;
        return &reference;
    }
};

#endif
