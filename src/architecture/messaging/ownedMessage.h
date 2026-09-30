/*
 * Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 * Distributed under the ISC license; see LICENSE.
 */

#ifndef OWNED_MESSAGE_H
#define OWNED_MESSAGE_H

#include "architecture/messaging/messaging.h"
#include <memory>
#include <vector>

/**
 * @brief Append a new owned message and a borrowed pointer to it.
 * @tparam Payload Message payload type.
 * @param owners Storage that owns the messages and controls their lifetime.
 * @param views Borrowed message pointers exposed to consumers.
 * @note If message construction or either insertion throws, both vectors retain
 * their previous sizes and entries. Vector capacity may change. Exceptions are
 * propagated after releasing any newly created message.
 * @note Existing message addresses remain stable when either vector grows. The
 * owner must outlive all uses of its borrowed messages. The views may contain
 * only a subset of the messages in owners, as in nested output collections.
 */
template<typename Payload>
void
addOwnedMessage(std::vector<std::unique_ptr<Message<Payload>>>& owners, std::vector<Message<Payload>*>& views)
{
    owners.push_back(std::make_unique<Message<Payload>>());
    try {
        views.push_back(owners.back().get());
    } catch (...) {
        owners.pop_back();
        throw;
    }
}

#endif // OWNED_MESSAGE_H
