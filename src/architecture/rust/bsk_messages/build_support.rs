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

use bindgen::callbacks::{ItemInfo, ItemKind, ParseCallbacks};

/// Rename only generated message payload types to Basilisk's Rust API names.
///
/// Bindgen invokes ``item_name`` for declarations, not structure fields. This
/// preserves field names supplied by message authors, including ``payload``
/// and ``data``, while exposing ``FooMsgPayload`` as ``FooMsg``.
#[derive(Debug)]
pub struct MessageBindingCallbacks;

impl ParseCallbacks for MessageBindingCallbacks {
    fn item_name(&self, item: ItemInfo<'_>) -> Option<String> {
        if !matches!(item.kind, ItemKind::Type) {
            return None;
        }
        item.name
            .strip_suffix("MsgPayload")
            .map(|prefix| format!("{prefix}Msg"))
    }
}

#[cfg(test)]
mod tests {
    use super::MessageBindingCallbacks;

    /// Preserve custom payload fields while renaming the containing payload type.
    #[test]
    fn payload_and_data_fields_remain_distinct() {
        let bindings = bindgen::Builder::default()
            .formatter(bindgen::Formatter::Prettyplease)
            .header_contents(
                "CollisionMsgPayload.h",
                "typedef struct {\n\
                     double payload;\n\
                     double data;\n\
                 } CollisionMsgPayload;\n\
                 typedef struct {\n\
                     CollisionMsgPayload payload;\n\
                     CollisionMsgPayload *payloadPointer;\n\
                 } CollisionMsg_C;\n",
            )
            .allowlist_type("CollisionMsgPayload|CollisionMsg_C")
            .parse_callbacks(Box::new(MessageBindingCallbacks))
            .generate()
            .expect("bindgen should generate the collision payload fixture")
            .to_string();

        assert!(bindings.contains("pub struct CollisionMsg"));
        assert!(bindings.contains("pub payload: f64"));
        assert!(bindings.contains("pub data: f64"));
        assert!(bindings.contains("pub payload: CollisionMsg"));
        assert!(bindings.contains("pub payloadPointer: *mut CollisionMsg"));
    }
}
