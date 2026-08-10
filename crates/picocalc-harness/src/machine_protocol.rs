//! Pure protocol helpers for the NEXT-4 headless machine API.
//!
//! This module deliberately knows nothing about the emulator or any
//! operation-specific payload.  It validates the schema-1 request envelope,
//! preserves an unknown operation name for the dispatcher, and emits one-line
//! deterministic JSON responses.  A request which fails here must not reach
//! the stateful machine-session layer.

use serde_json::{Map, Value};

/// The only machine API schema understood by this implementation.
pub const MACHINE_API_SCHEMA: u64 = 1;

/// Request correlation identifier.  String and integer IDs remain distinct;
/// in particular, integer `7` is never converted to string `"7"`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RequestId {
    String(String),
    Integer(u64),
}

impl RequestId {
    fn to_value(&self) -> Value {
        match self {
            Self::String(value) => Value::String(value.clone()),
            Self::Integer(value) => Value::Number((*value).into()),
        }
    }
}

/// Fields common to every schema-1 request.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RequestHeader {
    pub id: RequestId,
    /// Operation validity is a dispatcher concern.  Even an unknown or empty
    /// string is retained here instead of being reclassified by the parser.
    pub op: String,
}

/// Codes fixed by the schema-1 contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorCode {
    InvalidJson,
    InvalidRequest,
    UnsupportedOperation,
    UnsupportedObservation,
    MachineStopped,
    ModelError,
    EventOverflow,
}

impl ErrorCode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::InvalidJson => "invalid_json",
            Self::InvalidRequest => "invalid_request",
            Self::UnsupportedOperation => "unsupported_operation",
            Self::UnsupportedObservation => "unsupported_observation",
            Self::MachineStopped => "machine_stopped",
            Self::ModelError => "model_error",
            Self::EventOverflow => "event_overflow",
        }
    }
}

/// A stable-code protocol failure with a human-readable explanation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProtocolError {
    pub code: ErrorCode,
    pub message: String,
}

impl ProtocolError {
    pub fn new(code: ErrorCode, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }

    fn invalid_request(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::InvalidRequest, message)
    }
}

/// Parse one JSON line without touching machine state.
pub fn parse_request_line(line: &str) -> Result<Value, ProtocolError> {
    serde_json::from_str(line).map_err(|error| {
        ProtocolError::new(ErrorCode::InvalidJson, format!("invalid JSON: {error}"))
    })
}

/// Recover a valid correlation ID even when another envelope field is bad.
///
/// This lets an `invalid_request` response still echo the request ID when, for
/// example, `op` is missing.  Invalid JSON, a non-object root, a missing ID, or
/// an ID of the wrong type returns `None`, which maps to `"id":null`.
pub fn correlation_id(request: &Value) -> Option<RequestId> {
    match request.as_object()?.get("id")? {
        Value::String(value) => Some(RequestId::String(value.clone())),
        Value::Number(value) => value.as_u64().map(RequestId::Integer),
        _ => None,
    }
}

/// Validate and extract the common request envelope from a JSON value.
///
/// Operation-specific fields and operation-name validity intentionally remain
/// the dispatcher's responsibility.  Call [`reject_unknown_top_level_fields`]
/// with that operation's allow-list before mutating the machine.
pub fn parse_request_header(request: &Value) -> Result<RequestHeader, ProtocolError> {
    let object = request
        .as_object()
        .ok_or_else(|| ProtocolError::invalid_request("request root must be an object"))?;

    match object.get("schema") {
        None => {
            return Err(ProtocolError::invalid_request(
                "missing required field 'schema'",
            ));
        }
        Some(Value::Number(number)) if number.as_u64() == Some(MACHINE_API_SCHEMA) => {}
        Some(_) => {
            return Err(ProtocolError::invalid_request(format!(
                "field 'schema' must be integer {MACHINE_API_SCHEMA}"
            )));
        }
    }

    let id = match object.get("id") {
        None => {
            return Err(ProtocolError::invalid_request(
                "missing required field 'id'",
            ));
        }
        Some(_) => correlation_id(request).ok_or_else(|| {
            ProtocolError::invalid_request("field 'id' must be a string or a non-negative integer")
        })?,
    };

    let op = match object.get("op") {
        None => {
            return Err(ProtocolError::invalid_request(
                "missing required field 'op'",
            ));
        }
        Some(Value::String(value)) => value.clone(),
        Some(_) => {
            return Err(ProtocolError::invalid_request(
                "field 'op' must be a string",
            ));
        }
    };

    Ok(RequestHeader { id, op })
}

/// Return unknown top-level fields in lexical order.
///
/// `operation_fields` contains only fields specific to the selected operation;
/// the common `schema`, `id`, and `op` fields are always allowed.  Returning the
/// complete list lets a dispatcher report every typo without partially
/// executing a request.
pub fn unknown_top_level_fields(
    request: &Value,
    operation_fields: &[&str],
) -> Result<Vec<String>, ProtocolError> {
    let object = request
        .as_object()
        .ok_or_else(|| ProtocolError::invalid_request("request root must be an object"))?;
    let mut unknown = object
        .keys()
        .filter(|field| {
            !matches!(field.as_str(), "schema" | "id" | "op")
                && !operation_fields.contains(&field.as_str())
        })
        .cloned()
        .collect::<Vec<_>>();
    unknown.sort();
    Ok(unknown)
}

/// Fail closed when an operation request contains any field outside its
/// allow-list.
pub fn reject_unknown_top_level_fields(
    request: &Value,
    operation_fields: &[&str],
) -> Result<(), ProtocolError> {
    let unknown = unknown_top_level_fields(request, operation_fields)?;
    if unknown.is_empty() {
        return Ok(());
    }
    Err(ProtocolError::invalid_request(format!(
        "unknown top-level field(s): {}",
        unknown.join(", ")
    )))
}

/// Serialize a successful schema-1 response as exactly one JSONL record.
pub fn success_response_line(
    id: &RequestId,
    cycle: u64,
    result: Value,
    events: Vec<Value>,
) -> String {
    let mut response = Map::new();
    response.insert("schema".to_string(), Value::from(MACHINE_API_SCHEMA));
    response.insert("id".to_string(), id.to_value());
    response.insert("ok".to_string(), Value::Bool(true));
    response.insert("cycle".to_string(), Value::from(cycle));
    response.insert("result".to_string(), result);
    response.insert("events".to_string(), Value::Array(events));
    json_line(Value::Object(response))
}

/// Serialize a failed schema-1 response as exactly one JSONL record.
///
/// Invalid JSON or a missing/invalid request ID cannot be echoed.  Callers use
/// `None` in that pre-correlation case, producing `"id":null`; once an ID has
/// been validated they pass it through unchanged.
pub fn error_response_line(id: Option<&RequestId>, cycle: u64, error: &ProtocolError) -> String {
    let mut detail = Map::new();
    detail.insert(
        "code".to_string(),
        Value::String(error.code.as_str().to_string()),
    );
    detail.insert("message".to_string(), Value::String(error.message.clone()));

    let mut response = Map::new();
    response.insert("schema".to_string(), Value::from(MACHINE_API_SCHEMA));
    response.insert(
        "id".to_string(),
        id.map(RequestId::to_value).unwrap_or(Value::Null),
    );
    response.insert("ok".to_string(), Value::Bool(false));
    response.insert("cycle".to_string(), Value::from(cycle));
    response.insert("error".to_string(), Value::Object(detail));
    json_line(Value::Object(response))
}

fn json_line(value: Value) -> String {
    let mut line = serde_json::to_string(&value).expect("JSON Value serialization is infallible");
    line.push('\n');
    line
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn string_and_integer_ids_are_preserved_without_coercion() {
        let string = parse_request_header(&json!({"schema": 1, "id": "7", "op": "future"}))
            .expect("valid string ID");
        let integer = parse_request_header(&json!({"schema": 1, "id": 7, "op": "future"}))
            .expect("valid integer ID");

        assert_eq!(string.id, RequestId::String("7".to_string()));
        assert_eq!(integer.id, RequestId::Integer(7));
        assert_eq!(string.op, "future");
    }

    #[test]
    fn unknown_operation_is_left_for_the_dispatcher() {
        let header = parse_request_header(&json!({"schema": 1, "id": 0, "op": ""}))
            .expect("an operation string belongs to the dispatcher");
        assert_eq!(header.op, "");
    }

    #[test]
    fn malformed_json_has_the_stable_invalid_json_code() {
        let error = parse_request_line("{not-json").expect_err("must reject malformed JSON");
        assert_eq!(error.code, ErrorCode::InvalidJson);
        assert!(error.message.starts_with("invalid JSON:"));
    }

    #[test]
    fn valid_id_can_be_echo_correlated_when_another_field_is_bad() {
        let missing_op = json!({"schema": 1, "id": "request-4"});
        assert_eq!(
            correlation_id(&missing_op),
            Some(RequestId::String("request-4".to_string()))
        );
        assert_eq!(correlation_id(&json!({"id": -1})), None);
        assert_eq!(correlation_id(&json!([])), None);
    }

    #[test]
    fn request_root_and_required_fields_are_fail_closed() {
        for (request, message) in [
            (json!([]), "request root must be an object"),
            (
                json!({"id": "r", "op": "observe"}),
                "missing required field 'schema'",
            ),
            (
                json!({"schema": 1, "op": "observe"}),
                "missing required field 'id'",
            ),
            (
                json!({"schema": 1, "id": "r"}),
                "missing required field 'op'",
            ),
        ] {
            let error = parse_request_header(&request).expect_err("must reject invalid envelope");
            assert_eq!(error.code, ErrorCode::InvalidRequest);
            assert_eq!(error.message, message);
        }
    }

    #[test]
    fn schema_must_be_the_integer_one() {
        for schema in [json!(0), json!(2), json!(1.0), json!("1"), json!(true)] {
            let request = json!({"schema": schema, "id": "r", "op": "observe"});
            let error = parse_request_header(&request).expect_err("must reject wrong schema");
            assert_eq!(error.code, ErrorCode::InvalidRequest);
            assert_eq!(error.message, "field 'schema' must be integer 1");
        }
    }

    #[test]
    fn id_rejects_negative_fractional_and_other_types() {
        for id in [
            json!(-1),
            json!(1.5),
            json!(null),
            json!(true),
            json!([]),
            json!({}),
        ] {
            let request = json!({"schema": 1, "id": id, "op": "observe"});
            let error = parse_request_header(&request).expect_err("must reject invalid ID");
            assert_eq!(error.code, ErrorCode::InvalidRequest);
            assert_eq!(
                error.message,
                "field 'id' must be a string or a non-negative integer"
            );
        }
    }

    #[test]
    fn operation_must_be_a_string() {
        let request = json!({"schema": 1, "id": "r", "op": 7});
        let error = parse_request_header(&request).expect_err("must reject non-string op");
        assert_eq!(error.code, ErrorCode::InvalidRequest);
        assert_eq!(error.message, "field 'op' must be a string");
    }

    #[test]
    fn unknown_field_helper_adds_common_fields_and_sorts_failures() {
        let request = json!({
            "schema": 1,
            "id": "r",
            "op": "run",
            "max_cycles": 10,
            "z_typo": 1,
            "a_typo": 2
        });
        assert_eq!(
            unknown_top_level_fields(&request, &["max_cycles"]).expect("object"),
            ["a_typo", "z_typo"]
        );
        let error = reject_unknown_top_level_fields(&request, &["max_cycles"])
            .expect_err("unknown fields must fail");
        assert_eq!(error.code, ErrorCode::InvalidRequest);
        assert_eq!(error.message, "unknown top-level field(s): a_typo, z_typo");
    }

    #[test]
    fn operation_field_allow_list_accepts_a_complete_request() {
        let request = json!({
            "schema": 1,
            "id": 4,
            "op": "run_until",
            "condition": {"kind": "cycle_at_least", "cycle": 20},
            "max_cycles": 100,
            "poll_cycles": 5
        });
        reject_unknown_top_level_fields(&request, &["condition", "max_cycles", "poll_cycles"])
            .expect("all fields are known");
    }

    #[test]
    fn success_response_is_one_line_and_keeps_id_type() {
        let line = success_response_line(
            &RequestId::Integer(9),
            1234,
            json!({"advanced_cycles": 12}),
            vec![json!({"sequence": 1, "domain": "uart"})],
        );
        assert!(line.ends_with('\n'));
        assert_eq!(line.matches('\n').count(), 1);
        let value: Value = serde_json::from_str(&line).expect("valid response JSON");
        assert_eq!(value["schema"], 1);
        assert_eq!(value["id"], 9);
        assert_eq!(value["ok"], true);
        assert_eq!(value["cycle"], 1234);
        assert_eq!(value["result"]["advanced_cycles"], 12);
        assert_eq!(value["events"][0]["sequence"], 1);
    }

    #[test]
    fn error_response_is_one_line_with_stable_code_and_nullable_id() {
        let error = ProtocolError::new(
            ErrorCode::UnsupportedObservation,
            "LCD model is not attached\nno observation was made",
        );
        let line = error_response_line(None, 88, &error);
        assert!(line.ends_with('\n'));
        assert_eq!(line.matches('\n').count(), 1);
        let value: Value = serde_json::from_str(&line).expect("valid response JSON");
        assert_eq!(value["schema"], 1);
        assert!(value["id"].is_null());
        assert_eq!(value["ok"], false);
        assert_eq!(value["cycle"], 88);
        assert_eq!(value["error"]["code"], "unsupported_observation");
        assert_eq!(
            value["error"]["message"],
            "LCD model is not attached\nno observation was made"
        );
    }

    #[test]
    fn every_contract_error_code_has_its_stable_spelling() {
        assert_eq!(ErrorCode::InvalidJson.as_str(), "invalid_json");
        assert_eq!(ErrorCode::InvalidRequest.as_str(), "invalid_request");
        assert_eq!(
            ErrorCode::UnsupportedOperation.as_str(),
            "unsupported_operation"
        );
        assert_eq!(
            ErrorCode::UnsupportedObservation.as_str(),
            "unsupported_observation"
        );
        assert_eq!(ErrorCode::MachineStopped.as_str(), "machine_stopped");
        assert_eq!(ErrorCode::ModelError.as_str(), "model_error");
        assert_eq!(ErrorCode::EventOverflow.as_str(), "event_overflow");
    }
}
