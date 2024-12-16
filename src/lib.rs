// Code is ported from:
// https://github.com/ros2/rosidl/blob/master/rosidl_adapter/rosidl_adapter/parser.py
//
// Original license:
//
// Copyright 2014-2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Tests are ported from:
// https://github.com/foxglove/rosmsg/blob/main/src/parse.ros2.test.ts
//
// Original license:
//
//   Copyright 2018-2021 Cruise LLC
//
//   This source code is licensed under the Apache License, Version 2.0,
//   found at http://www.apache.org/licenses/LICENSE-2.0
//   You may not use this file except in compliance with the License.

use std::{fmt::Display, path::Path};
use anyhow::Context;
use derivative::Derivative;

const PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR: &str = "/";
const COMMENT_DELIMITER: &str = "#";
const CONSTANT_SEPARATOR: &str = "=";
const ARRAY_UPPER_BOUND_TOKEN: &str = "<=";
const STRING_UPPER_BOUND_TOKEN: &str = "<=";

const SERVICE_REQUEST_RESPONSE_SEPARATOR: &str = "---";
const SERVICE_REQUEST_MESSAGE_SUFFIX: &str = "_Request";
const SERVICE_RESPONSE_MESSAGE_SUFFIX: &str = "_Response";
// const SERVICE_EVENT_MESSAGE_SUFFIX: &str = "_Event";

const ACTION_REQUEST_RESPONSE_SEPARATOR: &str = "---";
const ACTION_GOAL_SUFFIX: &str = "_Goal";
const ACTION_RESULT_SUFFIX: &str = "_Result";
const ACTION_FEEDBACK_SUFFIX: &str = "_Feedback";

const ACTION_GOAL_SERVICE_SUFFIX: &str = "_Goal";
const ACTION_RESULT_SERVICE_SUFFIX: &str = "_Result";
const ACTION_FEEDBACK_MESSAGE_SUFFIX: &str = "_Feedback";

const PRIMITIVE_TYPES: &[&str] = &[
    "bool",
    "byte",
    "char",
    // TODO reconsider wchar
    "float32",
    "float64",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "string",
    "wstring",
    // TODO duration and time
    "duration",  // for compatibility only
    "time",  // for compatibility only
];

#[derive(Debug, Clone, PartialEq)]
pub enum PrimitiveType {
    Bool(bool),
    Float32(f32),
    Float64(f64),
    Int8(i8),
    UInt8(u8),
    Int16(i16),
    UInt16(u16),
    Int32(i32),
    UInt32(u32),
    Int64(i64),
    UInt64(u64),
    String(String),
}

impl Display for PrimitiveType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PrimitiveType::Bool(value) => write!(f, "{}", value),
            PrimitiveType::Float32(value) => write!(f, "{}", value),
            PrimitiveType::Float64(value) => write!(f, "{}", value),
            PrimitiveType::Int8(value) => write!(f, "{}", value),
            PrimitiveType::UInt8(value) => write!(f, "{}", value),
            PrimitiveType::Int16(value) => write!(f, "{}", value),
            PrimitiveType::UInt16(value) => write!(f, "{}", value),
            PrimitiveType::Int32(value) => write!(f, "{}", value),
            PrimitiveType::UInt32(value) => write!(f, "{}", value),
            PrimitiveType::Int64(value) => write!(f, "{}", value),
            PrimitiveType::UInt64(value) => write!(f, "{}", value),
            PrimitiveType::String(value) => {
                if !value.contains('\'') {
                    write!(f, "'{}'", value.replace("'", "\\'"))
                } else {
                    write!(f, "\"{}\"", value.replace("\"", "\\\""))
                }
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PrimitiveTypeOrList {
    Single(PrimitiveType),
    List(Vec<PrimitiveType>),
}

impl Display for PrimitiveTypeOrList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PrimitiveTypeOrList::Single(value) => write!(f, "{}", value),
            PrimitiveTypeOrList::List(values) => {
                let values = values.iter().map(|v| v.to_string()).collect::<Vec<String>>().join(", ");
                write!(f, "[{}]", values)
            }
        }
    }
}

#[derive(Debug, Clone, Default)]
struct Annotations {
    comment: Vec<String>,
    // unit: String,
}

fn check_valid_package_name(name: &str) -> anyhow::Result<()> {
    // static VALID_PACKAGE_NAME_PATTERN: Lazy<Regex> = Lazy::new(|| Regex::new(concat!(
    //     "^",
    //     "(?!.*__)",  // no consecutive underscores
    //     "(?!.*_$)",  // no underscore at the end
    //     "[a-z]",  // first character must be alpha
    //     "[a-z0-9_]*",  // followed by alpha, numeric, and underscore
    //     "$"
    // )).unwrap());

    let Some(first_char) = name.chars().next() else {
        anyhow::bail!("name must not be empty");
    };

    if !first_char.is_ascii_lowercase() {
        anyhow::bail!("the first character of '{}' must be alpha", name);
    }

    let mut last_ch = None;
    for c in name.chars() {
        if !c.is_ascii_lowercase() && !c.is_ascii_digit() && c != '_' {
            anyhow::bail!("invalid character '{}' in '{}', must be lowercase, numeric, or underscore", c, name);
        }
        if last_ch == Some('_') && c == '_' {
            anyhow::bail!("consecutive underscores in '{}'", name);
        }
        last_ch = Some(c);
    }

    if let Some(last_ch) = last_ch {
        if last_ch == '_' {
            anyhow::bail!("the last character of '{}' must not be an underscore", name);
        }
    }

    Ok(())
}

fn check_valid_field_name(name: &str) -> anyhow::Result<()> {
    check_valid_package_name(name)
}

fn check_valid_message_name(mut name: &str) -> anyhow::Result<()> {
    if let Some(prefix) = name.strip_prefix("Sample_") {
        name = prefix;
    }
    for suffix in &[
        SERVICE_REQUEST_MESSAGE_SUFFIX,
        SERVICE_RESPONSE_MESSAGE_SUFFIX,
        ACTION_GOAL_SERVICE_SUFFIX,
        ACTION_RESULT_SERVICE_SUFFIX,
        ACTION_FEEDBACK_MESSAGE_SUFFIX,
    ] {
        if let Some(suffix) = name.strip_suffix(suffix) {
            name = suffix;
        }
    }

    //static VALID_MESSAGE_NAME_PATTERN: Lazy<Regex> = Lazy::new(|| Regex::new("^[A-Z][A-Za-z0-9]*$").unwrap());

    let Some(first_char) = name.chars().next() else {
        anyhow::bail!("name must not be empty");
    };

    if !first_char.is_ascii_uppercase() {
        anyhow::bail!("the first character of '{}' must be uppercase", name);
    }

    for c in name.chars() {
        if !c.is_ascii_alphanumeric() {
            anyhow::bail!("invalid character '{}' in '{}', must be alphanumeric", c, name);
        }
    }

    Ok(())
}

fn check_valid_constant_name(name: &str) -> anyhow::Result<()> {
    // static VALID_CONSTANT_NAME_PATTERN: Lazy<Regex> = Lazy::new(|| Regex::new("^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$").unwrap());

    let Some(first_char) = name.chars().next() else {
        anyhow::bail!("name must not be empty");
    };

    if !first_char.is_ascii_uppercase() {
        anyhow::bail!("the first character of '{}' must be uppercase", name);
    }

    let mut last_ch = None;

    for c in name.chars() {
        if !c.is_ascii_alphanumeric() && c != '_' {
            anyhow::bail!("invalid character '{}' in '{}', must be uppercase, numeric, or underscore", c, name);
        }
        if last_ch == Some('_') && c == '_' {
            anyhow::bail!("consecutive underscores in '{}'", name);
        }
        last_ch = Some(c);
    }

    if let Some(last_ch) = last_ch {
        if last_ch == '_' {
            anyhow::bail!("the last character of '{}' must not be an underscore", name);
        }
    }

    Ok(())
}

#[derive(Debug, Clone, PartialEq, Hash)]
pub struct BaseType {
    pkg_name: Option<String>,
    type_: String,
    string_upper_bound: Option<usize>,
}

impl BaseType {
    pub fn new(type_string: &str, context_package_name: Option<&str>) -> anyhow::Result<Self> {
        // check for primitive types
        if PRIMITIVE_TYPES.contains(&type_string) {
            return Ok(BaseType {
                pkg_name: None,
                type_: type_string.to_string(),
                string_upper_bound: None,
            });
        }

        if type_string.starts_with("string") || type_string.starts_with("wstring") {
            let (type_, upper_bound_string) = type_string.split_once(STRING_UPPER_BOUND_TOKEN).context("invalid string type")?;

            let string_upper_bound = (|| {
                let string_upper_bound = upper_bound_string.parse::<usize>().ok()?;
                if string_upper_bound == 0 {
                    None
                } else {
                    Some(string_upper_bound)
                }
            })().with_context(||
                format!("the upper bound of the string type '{}' must be a valid integer value > 0", type_string)
            )?;

            return Ok(BaseType {
                pkg_name: None,
                type_: type_.to_string(),
                string_upper_bound: Some(string_upper_bound),
            });
        }

        // split primitive type information
        let parts = type_string.split(PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR).collect::<Vec<&str>>();
        if !(parts.len() == 2 || (parts.len() == 1 && context_package_name.is_some())) {
            anyhow::bail!(format!("invalid resource name: {}", type_string));
        }

        let (pkg_name, type_) = if parts.len() == 2 {
            // either the type string contains the package name
            (parts[0].to_string(), parts[1].to_string())
        } else if let Some(context_package_name) = context_package_name {
            // or the package name is provided by context
            (context_package_name.to_string(), type_string.to_string())
        } else {
            anyhow::bail!("either parts has length 2 or context_package_name exist otherwise BaseType Malformed");
        };

        check_valid_package_name(&pkg_name)?;
        check_valid_message_name(&type_)?;

        Ok(BaseType {
            pkg_name: Some(pkg_name),
            type_,
            string_upper_bound: None,
        })
    }

    pub fn type_(&self) -> &str {
        &self.type_
    }

    pub fn string_upper_bound(&self) -> Option<usize> {
        self.string_upper_bound
    }

    pub fn is_primitive_type(&self) -> bool {
        self.pkg_name.is_none()
    }
}

impl Display for BaseType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(pkg_name) = &self.pkg_name {
            write!(f, "{}/{}", pkg_name, self.type_)
        } else if let Some(string_upper_bound) = self.string_upper_bound {
            write!(f, "{}{}{}", self.type_, STRING_UPPER_BOUND_TOKEN, string_upper_bound)
        } else {
            write!(f, "{}", self.type_)
        }
    }
}

#[derive(Debug, Clone, PartialEq, Hash)]
pub struct Type {
    base: BaseType,
    is_array: bool,
    array_size: Option<usize>,
    is_upper_bound: bool,
}

impl Type {
    pub fn new(type_string: &str, context_package_name: Option<&str>) -> anyhow::Result<Self> {
        // check for array brackets
        let is_array = type_string.ends_with(']');
        let mut type_string = type_string;
        let mut array_size = None;
        let mut is_upper_bound = false;

        if is_array {
            let index = type_string.rfind('[')
                .with_context(|| format!("the type {} ends with ']' but does not contain a '['", type_string))?;

            let array_size_string = &type_string[index + 1..type_string.len() - 1];
            // get array limit
            if !array_size_string.is_empty() {
                // check if the limit is an upper bound
                is_upper_bound = array_size_string.starts_with(ARRAY_UPPER_BOUND_TOKEN);
                let array_size_string = if is_upper_bound {
                    &array_size_string[ARRAY_UPPER_BOUND_TOKEN.len()..]
                } else {
                    array_size_string
                };

                array_size = Some((|| {
                    let array_size = array_size_string.parse::<usize>().ok()?;
                    // check valid range
                    if array_size == 0 {
                        None
                    } else {
                        Some(array_size)
                    }
                })().with_context(||
                    format!(
                        "the size of array type '{}' must be a valid integer value > 0 optionally prefixed with '{}' if it is only an upper bound",
                        ARRAY_UPPER_BOUND_TOKEN,
                        type_string,
                    )
                )?);
            };

            type_string = &type_string[..index];
        };

        Ok(Type {
            base: BaseType::new(type_string, context_package_name)?,
            is_array,
            array_size,
            is_upper_bound,
        })
    }

    pub fn base(&self) -> &BaseType {
        &self.base
    }

    pub fn is_array(&self) -> bool {
        self.is_array
    }

    pub fn is_dynamic_array(&self) -> bool {
        self.is_array && (self.array_size.is_none() || self.is_upper_bound)
    }

    pub fn is_fixed_size_array(&self) -> bool {
        self.is_array && self.array_size.is_some() && !self.is_upper_bound
    }

    pub fn array_size(&self) -> Option<usize> {
        self.array_size
    }

    pub fn is_upper_bound(&self) -> bool {
        self.is_upper_bound
    }
}

impl Display for Type {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.base)?;
        if self.is_array {
            write!(f, "[")?;
            if self.is_upper_bound {
                write!(f, "{}", ARRAY_UPPER_BOUND_TOKEN)?;
            }
            if let Some(array_size) = self.array_size {
                write!(f, "{}", array_size)?;
            }
            write!(f, "]")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
#[derive(Derivative)]
#[derivative(PartialEq)]
pub struct Constant {
    type_: String,
    name: String,
    value: PrimitiveType,
    #[derivative(PartialEq = "ignore")]
    annotations: Annotations,
}

impl Constant {
    pub fn new(primitive_type: &str, name: &str, value_string: &str) -> anyhow::Result<Self> {
        if !PRIMITIVE_TYPES.contains(&primitive_type) {
            anyhow::bail!("the constant type '{}' must be a primitive type", primitive_type);
        }

        check_valid_constant_name(name)?;

        let value = parse_primitive_value_string(&Type::new(primitive_type, None)?, value_string)?;

        Ok(Constant {
            type_: primitive_type.to_string(),
            name: name.to_string(),
            value,
            annotations: Default::default(),
        })
    }

    pub fn type_(&self) -> &str {
        &self.type_
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn value(&self) -> &PrimitiveType {
        &self.value
    }

    pub fn annotations(&self) -> &[String] {
        &self.annotations.comment
    }
}

impl Display for Constant {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} {}={}", self.type_, self.name, self.value)
    }
}


#[derive(Debug, Clone)]
#[derive(Derivative)]
#[derivative(PartialEq)]
pub struct Field {
    type_: Type,
    name: String,
    default_value: Option<PrimitiveTypeOrList>,
    #[derivative(PartialEq = "ignore")]
    annotations: Annotations,
}

impl Field {
    pub fn new(type_: Type, name: &str, default_value_string: Option<&str>) -> anyhow::Result<Self> {
        check_valid_field_name(name)?;

        let default_value = match default_value_string {
            Some(default_value_string) => {
                Some(parse_value_string(&type_, default_value_string)?)
            },
            None => None,
        };

        Ok(Field {
            type_,
            name: name.to_string(),
            default_value,
            annotations: Default::default(),
        })
    }

    pub fn type_(&self) -> &Type {
        &self.type_
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn default_value(&self) -> Option<&PrimitiveTypeOrList> {
        self.default_value.as_ref()
    }

    pub fn annotations(&self) -> &[String] {
        &self.annotations.comment
    }
}

impl Display for Field {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} {}", self.type_, self.name)?;
        if let Some(default_value) = &self.default_value {
            write!(f, " {}", default_value)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
#[derive(Derivative)]
#[derivative(PartialEq)]
pub struct MessageSpecification {
    base_type: BaseType,
    #[derivative(PartialEq = "ignore")]
    msg_name: String,
    #[derivative(PartialEq = "ignore")]
    annotations: Annotations,
    fields: Vec<Field>,
    constants: Vec<Constant>,
}

impl MessageSpecification {
    pub fn new(pkg_name: &str, msg_name: &str, fields: Vec<Field>, constants: Vec<Constant>) -> anyhow::Result<Self> {
        let base_type = BaseType::new(
            &format!("{}{}{}", pkg_name, PACKAGE_NAME_MESSAGE_TYPE_SEPARATOR, msg_name),
            None,
        )?;

        Ok(MessageSpecification {
            base_type,
            msg_name: msg_name.to_string(),
            annotations: Default::default(),
            fields,
            constants,
        })
    }

    pub fn base_type(&self) -> &BaseType {
        &self.base_type
    }

    pub fn msg_name(&self) -> &str {
        &self.msg_name
    }

    pub fn annotations(&self) -> &[String] {
        &self.annotations.comment
    }

    pub fn fields(&self) -> &[Field] {
        &self.fields
    }

    pub fn constants(&self) -> &[Constant] {
        &self.constants
    }
}

impl Display for MessageSpecification {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "# {}", self.base_type)?;
        for constant in &self.constants {
            writeln!(f, "{}", constant)?;
        }
        for field in &self.fields {
            writeln!(f, "{}", field)?;
        }
        Ok(())
    }
}

pub fn parse_message_file(pkg_name: &str, interface_filename: &str) -> anyhow::Result<MessageSpecification> {
    let basename = Path::new(interface_filename).file_name().and_then(|s| s.to_str()).context("invalid filename")?;
    let msg_name = Path::new(basename).file_stem().and_then(|s| s.to_str()).context("invalid filename")?;
    let message_string = std::fs::read_to_string(interface_filename).context("failed to read file")?;
    parse_message_string(pkg_name, msg_name, &message_string)
}

fn extract_file_level_comments(message_string: &str) -> (Vec<String>, Vec<String>) {
    let lines = message_string.split('\n').collect::<Vec<&str>>();
    let index = lines.iter().position(|v| !v.starts_with(COMMENT_DELIMITER)).unwrap_or(lines.len());
    let file_level_comments = lines[..index].iter().map(|v| v.trim_start_matches(COMMENT_DELIMITER).to_string()).collect();
    let file_content = lines[index..].iter().map(|v| v.to_string()).collect();
    (file_level_comments, file_content)
}

pub fn parse_message_string(pkg_name: &str, msg_name: &str, message_string: &str) -> anyhow::Result<MessageSpecification> {
    enum LastElement {
        Field,
        Constant,
        None,
    }

    let mut fields = Vec::<Field>::new();
    let mut constants = Vec::<Constant>::new();
    let mut last_element = LastElement::None;
    // replace tabs with spaces
    let message_string = message_string.replace('\t', " ");

    let mut current_comments = vec![];
    let (message_comments, lines) = extract_file_level_comments(&message_string);

    for line in lines {
        let mut line = line.trim_end();

        // ignore empty lines
        if line.is_empty() {
            // file-level comments stop at the first empty line
            continue;
        }

        let index = line.find(COMMENT_DELIMITER);

        // comment
        let mut comment = None;
        if let Some(index) = index {
            comment = Some(line[index..].trim_start_matches(COMMENT_DELIMITER));
            line = line[..index].trim_end();
        }

        if let Some(comment) = comment {
            if line.is_empty() {
                // indented comment line
                // append to previous field / constant if available or ignore
                match last_element {
                    LastElement::Field => {
                        if let Some(last_element) = fields.last_mut() {
                            last_element.annotations.comment.push(comment.to_string());
                        }
                    },
                    LastElement::Constant => {
                        if let Some(last_element) = constants.last_mut() {
                            last_element.annotations.comment.push(comment.to_string());
                        }
                    },
                    LastElement::None => {},
                }
                continue;
            }
            // collect "unused" comments
            current_comments.push(comment.to_string());

            line = line.trim_end();
            if line.is_empty() {
                continue;
            }
        }

        let (type_string, rest) = line.split_once(' ')
            .map(|(type_string, rest)| (type_string, rest.trim_start()))
            .filter(|(_, rest)| !rest.is_empty())
            .with_context(|| format!("invalid line: {}", line))?;

        if let Some((name, value)) = rest.split_once(CONSTANT_SEPARATOR) {
            // line contains a constant
            let name = name.trim_end();
            let value = value.trim_start();
            constants.push(Constant::new(type_string, name, value)?);
            last_element = LastElement::Constant;
        } else {
            // line contains a field
            let (field_name, default_value_string) = rest.split_once(' ')
                .map(|(field_name, default_value_string)| (field_name, Some(default_value_string.trim_start())))
                .unwrap_or((rest, None));

            let type_ = Type::new(type_string, Some(pkg_name))?;
            let field = Field::new(type_, field_name, default_value_string)?;
            fields.push(field);

            last_element = LastElement::Field;
        }

        // add "unused" comments to the field / constant
        if let Some(last_element) = fields.last_mut() {
            last_element.annotations.comment.extend(std::mem::take(&mut current_comments));
        }
    }

    let mut msg = MessageSpecification::new(pkg_name, msg_name, fields, constants)?;
    msg.annotations.comment = message_comments;

    // condense comment lines, extract special annotations
    process_comments(&mut msg.annotations);
    for field in &mut msg.fields {
        process_comments(&mut field.annotations);
    }
    for constant in &mut msg.constants {
        process_comments(&mut constant.annotations);
    }

    Ok(msg)
}

fn process_comments(annotations: &mut Annotations) {
    if !annotations.comment.is_empty() {
        let mut lines = std::mem::take(&mut annotations.comment);

        // look for a unit in brackets
        // the unit should not contains a comma since it might be a range
        // let comment = lines.join("\n");
        // let pattern = r"(\s*\[([^,\]]+)\])";
        // let matches = regex::Regex::new(pattern).unwrap().find_iter(&comment).collect::<Vec<_>>();
        // if matches.len() == 1 {
        //     annotations.unit = matches[0].as_str().to_string();
        //     // remove the unit from the comment
        //     for line in &mut lines {
        //         *line = line.replace(matches[0].as_str(), "");
        //     }
        // }

        // remove empty leading lines
        while !lines.is_empty() && lines[0].is_empty() {
            lines.remove(0);
        }
        // remove empty trailing lines
        while !lines.is_empty() && lines[lines.len() - 1].is_empty() {
            lines.pop();
        }
        // remove consecutive empty lines
        let mut i = 1;
        while i < lines.len() {
            if lines[i].is_empty() && lines[i - 1].is_empty() {
                lines.drain(i - 1..=i);
                continue;
            }
            i += 1;
        }
        if !lines.is_empty() {
            let text = lines.join("\n");
            annotations.comment = text.split('\n').map(|s| s.to_string()).collect();
        }
    }
}

fn parse_value_string(type_: &Type, value_string: &str) -> anyhow::Result<PrimitiveTypeOrList> {
    if type_.base.is_primitive_type() && !type_.is_array {
        return Ok(PrimitiveTypeOrList::Single(parse_primitive_value_string(type_, value_string)?));
    }

    if type_.base.is_primitive_type() && type_.is_array {
        // check for array brackets
        if !value_string.starts_with('[') || !value_string.ends_with(']') {
            anyhow::bail!("array value must start with '[' and end with ']'");
        }
        let elements_string = &value_string[1..value_string.len() - 1];

        let value_strings = if type_.base.type_ == "string" || type_.base.type_ == "wstring" {
            // String arrays need special processing as the comma can be part of a quoted string
            // and not a separator of array elements
            parse_string_array_value_string(elements_string, type_.array_size)?
        } else {
            elements_string.split(',').map(|s| s.trim().to_owned()).collect::<Vec<_>>()
        };

        if let Some(array_size) = type_.array_size {
            // check for exact size
            if !type_.is_upper_bound && value_strings.len() != array_size {
                anyhow::bail!("array must have exactly {} elements, not {}", array_size, value_strings.len());
            }
            // check for upper bound
            if type_.is_upper_bound && value_strings.len() > array_size {
                anyhow::bail!("array must have not more than {} elements, not {}", array_size, value_strings.len());
            }
        }

        // parse all primitive values one by one
        let mut values = Vec::<PrimitiveType>::new();
        for (index, element_string) in value_strings.iter().enumerate() {
            (|| {
                let element_string = element_string.trim();
                let base_type = Type::new(&type_.base.type_, None)?;
                let value = parse_primitive_value_string(&base_type, element_string)?;
                values.push(value);
                anyhow::Ok(())
            })().with_context(|| format!("element {} is invalid", index))?;
        }

        return Ok(PrimitiveTypeOrList::List(values));
    }

    anyhow::bail!("parsing string values into type '{}' is not supported", type_);
}

fn parse_string_array_value_string(element_string: &str, _expected_size: Option<usize>) -> anyhow::Result<Vec<String>> {
    // Walks the string, if start with quote (' or ") find next unescaped quote,
    // returns a list of string elements
    let mut value_strings = Vec::<String>::new();
    let mut element_string = element_string.trim();

    while !element_string.is_empty() {
        element_string = element_string.trim_start();
        if element_string.starts_with(',') {
            anyhow::bail!("unexpected ',' at beginning of '{}'", element_string);
        }
        if element_string.is_empty() {
            return Ok(value_strings);
        }
        let mut quoted_value = false;
        for quote in ['"', '\''] {
            if element_string.starts_with(quote) {
                quoted_value = true;
                let end_quote_idx = find_matching_end_quote(element_string, quote);
                if let Some(end_quote_idx) = end_quote_idx {
                    let mut value_string = element_string[1..end_quote_idx + 1].to_string();
                    value_string = value_string.replace(&format!("\\{}", quote), &quote.to_string());
                    value_strings.push(value_string);
                    element_string = &element_string[end_quote_idx + 2..];
                } else {
                    anyhow::bail!("string '{}' incorrectly quoted", element_string);
                }
            }
        }

        if !quoted_value {
            let next_comma_idx = element_string.find(',');
            if let Some(next_comma_idx) = next_comma_idx {
                value_strings.push(element_string[..next_comma_idx].to_string());
                element_string = &element_string[next_comma_idx..];
            } else {
                value_strings.push(element_string.to_string());
                element_string = "";
            }
        }

        element_string = element_string.trim_start();
        element_string = element_string.strip_prefix(',').unwrap_or(element_string);
    }

    Ok(value_strings)
}

fn find_matching_end_quote(string: &str, quote: char) -> Option<usize> {
    let mut final_quote_idx = 0;
    let mut chars = string.chars().peekable();
    chars.next(); // skip the first quote
    while let Some(c) = chars.next() {
        if c == quote {
            return Some(final_quote_idx);
        }
        if c == '\\' {
            // Skip the next character if it's escaped
            if chars.peek() == Some(&quote) {
                chars.next();
            }
        }
        final_quote_idx += 1;
    }
    None
}

fn parse_primitive_value_string(type_: &Type, value_string: &str) -> anyhow::Result<PrimitiveType> {
    if !type_.base.is_primitive_type() || type_.is_array {
        anyhow::bail!("the passed type must be a non-array primitive type");
    }

    let primitive_type = &type_.base.type_;

    if primitive_type == "bool" {
        let true_values = ["true", "1"];
        let false_values = ["false", "0"];
        if !true_values.contains(&value_string.to_lowercase().as_str()) && !false_values.contains(&value_string.to_lowercase().as_str()) {
            anyhow::bail!("value '{}' must be either 'true' / '1' or 'false' / '0'", value_string);
        }
        return Ok(PrimitiveType::Bool(true_values.contains(&value_string.to_lowercase().as_str())));
    }

    if primitive_type == "byte" || primitive_type == "char" {
        // same as uint8
        let value = value_string.parse::<u8>()
            .with_context(|| format!("value '{}' must be a valid integer value >= 0 and <= 255", value_string))?;
        return Ok(PrimitiveType::UInt8(value));
    }

    if primitive_type == "float32" {
        let value = value_string.parse::<f32>()
            .with_context(|| format!("value '{}' must be a floating point number using '.' as the separator", value_string))?;
        return Ok(PrimitiveType::Float32(value));
    }

    if primitive_type == "float64" {
        let value = value_string.parse::<f64>()
            .with_context(|| format!("value '{}' must be a floating point number using '.' as the separator", value_string))?;
        return Ok(PrimitiveType::Float64(value));
    }

    if primitive_type == "int8" {
        let value = value_string.parse::<i8>()
            .with_context(|| format!("value '{}' must be a valid integer value >= -128 and <= 127", value_string))?;
        return Ok(PrimitiveType::Int8(value));
    }

    if primitive_type == "uint8" {
        let value = value_string.parse::<u8>()
            .with_context(|| format!("value '{}' must be a valid integer value >= 0 and <= 255", value_string))?;
        return Ok(PrimitiveType::UInt8(value));
    }

    if primitive_type == "int16" {
        let value = value_string.parse::<i16>()
            .with_context(|| format!("value '{}' must be a valid integer value >= -32768 and <= 32767", value_string))?;
        return Ok(PrimitiveType::Int16(value));
    }

    if primitive_type == "uint16" {
        let value = value_string.parse::<u16>()
            .with_context(|| format!("value '{}' must be a valid integer value >= 0 and <= 65535", value_string))?;
        return Ok(PrimitiveType::UInt16(value));
    }

    if primitive_type == "int32" {
        let value = value_string.parse::<i32>()
            .with_context(|| format!("value '{}' must be a valid integer value >= -2147483648 and <= 2147483647", value_string))?;
        return Ok(PrimitiveType::Int32(value));
    }

    if primitive_type == "uint32" {
        let value = value_string.parse::<u32>()
            .with_context(|| format!("value '{}' must be a valid integer value >= 0 and <= 4294967295", value_string))?;
        return Ok(PrimitiveType::UInt32(value));
    }

    if primitive_type == "int64" {
        let value = value_string.parse::<i64>()
            .with_context(|| format!("value '{}' must be a valid integer value >= -9223372036854775808 and <= 9223372036854775807", value_string))?;
        return Ok(PrimitiveType::Int64(value));
    }

    if primitive_type == "uint64" {
        let value = value_string.parse::<u64>()
            .with_context(|| format!("value '{}' must be a valid integer value >= 0 and <= 18446744073709551615", value_string))?;
        return Ok(PrimitiveType::UInt64(value));
    }

    if primitive_type == "string" || primitive_type == "wstring" {
        // remove outer quotes to allow leading / trailing spaces in the string
        let mut value_string = value_string.trim().to_string();
        for quote in ['"', '\''] {
            let Some(s) = value_string.strip_prefix(quote) else { continue; };
            let Some(s) = s.strip_suffix(quote) else { continue; };

            let mut escaped = false;
            let mut unescaped_value = String::new();
            for c in s.chars() {
                if escaped {
                    unescaped_value.push(c);
                    escaped = false;
                } else if c == '\\' {
                    escaped = true;
                } else {
                    unescaped_value.push(c);
                }
            }
            if escaped {
                anyhow::bail!("string inner quotes not properly escaped");
            }
            value_string = unescaped_value;

            break;
        }

        // check that value is in valid range
        if let Some(string_upper_bound) = type_.base.string_upper_bound {
            if value_string.len() > string_upper_bound {
                anyhow::bail!("string must not exceed the maximum length of {} characters", string_upper_bound);
            }
        }

        return Ok(PrimitiveType::String(value_string));
    }

    anyhow::bail!("unknown primitive type '{}'", primitive_type);
}

// def validate_field_types(spec: Union[MessageSpecification,
//                                      'ServiceSpecification',
//                                      'ActionSpecification'],
//                          known_msg_types: List[BaseType]) -> None:
//     if isinstance(spec, MessageSpecification):
//         spec_type = 'Message'
//         fields: List[Field] = spec.fields
//     elif isinstance(spec, ServiceSpecification):
//         spec_type = 'Service'
//         fields = spec.request.fields + spec.response.fields
//     elif isinstance(spec, ActionSpecification):
//         spec_type = 'Action'
//         fields = []
//         for service in [spec.goal_service, spec.result_service]:
//             fields += service.request.fields
//             fields += service.response.fields
//     else:
//         assert False, 'Unknown specification type: %s' % type(spec)
//     for field in fields:
//         if field.type.is_primitive_type():
//             continue
//         base_type = BaseType(BaseType.__str__(field.type))
//         if base_type not in known_msg_types:
//             raise UnknownMessageType(
//                 "%s interface '%s' contains an unknown field type: %s" %
//                 (spec_type, base_type, field))


#[derive(Debug, Clone)]
pub struct ServiceSpecification {
    pkg_name: String,
    srv_name: String,
    request: MessageSpecification,
    response: MessageSpecification,
}

impl ServiceSpecification {
    pub fn new(pkg_name: &str, srv_name: &str, request: MessageSpecification, response: MessageSpecification) -> Self {
        ServiceSpecification {
            pkg_name: pkg_name.to_string(),
            srv_name: srv_name.to_string(),
            request,
            response,
        }
    }

    pub fn pkg_name(&self) -> &str {
        &self.pkg_name
    }

    pub fn srv_name(&self) -> &str {
        &self.srv_name
    }

    pub fn request(&self) -> &MessageSpecification {
        &self.request
    }

    pub fn response(&self) -> &MessageSpecification {
        &self.response
    }
}

impl Display for ServiceSpecification {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "# {}/{}", self.pkg_name, self.srv_name)?;
        write!(f, "{}", self.request)?;
        writeln!(f, "---")?;
        write!(f, "{}", self.response)
    }
}

pub fn parse_service_file(pkg_name: &str, interface_filename: &str) -> anyhow::Result<ServiceSpecification> {
    let basename = Path::new(interface_filename).file_name().and_then(|s| s.to_str()).context("invalid filename")?;
    let srv_name = Path::new(basename).file_stem().and_then(|s| s.to_str()).context("invalid filename")?;
    let message_string = std::fs::read_to_string(interface_filename).context("failed to read file")?;
    parse_service_string(pkg_name, srv_name, &message_string)
}

pub fn parse_service_string(pkg_name: &str, srv_name: &str, message_string: &str) -> anyhow::Result<ServiceSpecification> {
    let lines = message_string.split('\n').collect::<Vec<&str>>();
    let separator_indices = lines.iter().enumerate()
        .filter(|(_, line)| **line == SERVICE_REQUEST_RESPONSE_SEPARATOR)
        .map(|(index, _)| index).collect::<Vec<_>>();

    if separator_indices.is_empty() {
        anyhow::bail!("Could not find separator '{}' between request and response", SERVICE_REQUEST_RESPONSE_SEPARATOR);
    }
    if separator_indices.len() != 1 {
        anyhow::bail!("Could not find unique separator '{}' between request and response", SERVICE_REQUEST_RESPONSE_SEPARATOR);
    }

    let request_message_string = lines[..separator_indices[0]].join("\n");
    let request_message = parse_message_string(pkg_name, &format!("{}{}", srv_name, SERVICE_REQUEST_MESSAGE_SUFFIX), &request_message_string)?;

    let response_message_string = lines[separator_indices[0] + 1..].join("\n");
    let response_message = parse_message_string(pkg_name, &format!("{}{}", srv_name, SERVICE_RESPONSE_MESSAGE_SUFFIX), &response_message_string)?;

    Ok(ServiceSpecification::new(pkg_name, srv_name, request_message, response_message))
}

#[derive(Debug, Clone)]
pub struct ActionSpecification {
    pkg_name: String,
    action_name: String,
    goal: MessageSpecification,
    result: MessageSpecification,
    feedback: MessageSpecification,
}

impl ActionSpecification {
    pub fn new(pkg_name: &str, action_name: &str, goal: MessageSpecification, result: MessageSpecification, feedback: MessageSpecification) -> Self {
        ActionSpecification {
            pkg_name: pkg_name.to_string(),
            action_name: action_name.to_string(),
            goal,
            result,
            feedback,
        }
    }

    pub fn pkg_name(&self) -> &str {
        &self.pkg_name
    }

    pub fn action_name(&self) -> &str {
        &self.action_name
    }

    pub fn goal(&self) -> &MessageSpecification {
        &self.goal
    }

    pub fn result(&self) -> &MessageSpecification {
        &self.result
    }

    pub fn feedback(&self) -> &MessageSpecification {
        &self.feedback
    }
}

impl Display for ActionSpecification {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "# {}/{}", self.pkg_name, self.action_name)?;
        write!(f, "{}", self.goal)?;
        writeln!(f, "---")?;
        write!(f, "{}", self.result)?;
        writeln!(f, "---")?;
        write!(f, "{}", self.feedback)
    }
}

pub fn parse_action_file(pkg_name: &str, interface_filename: &str) -> anyhow::Result<ActionSpecification> {
    let basename = Path::new(interface_filename).file_name().and_then(|s| s.to_str()).context("invalid filename")?;
    let action_name = Path::new(basename).file_stem().and_then(|s| s.to_str()).context("invalid filename")?;
    let message_string = std::fs::read_to_string(interface_filename).context("failed to read file")?;
    parse_action_string(pkg_name, action_name, &message_string)
}

pub fn parse_action_string(pkg_name: &str, action_name: &str, message_string: &str) -> anyhow::Result<ActionSpecification> {
    let lines = message_string.split('\n').collect::<Vec<&str>>();
    let separator_indices = lines.iter().enumerate()
        .filter(|(_, line)| **line == ACTION_REQUEST_RESPONSE_SEPARATOR)
        .map(|(index, _)| index).collect::<Vec<_>>();

    if separator_indices.len() != 2 {
        anyhow::bail!("Number of '{}' separators nonconformant with action definition", ACTION_REQUEST_RESPONSE_SEPARATOR);
    }

    let goal_string = lines[..separator_indices[0]].join("\n");
    let result_string = lines[separator_indices[0] + 1..separator_indices[1]].join("\n");
    let feedback_string = lines[separator_indices[1] + 1..].join("\n");

    let goal_message = parse_message_string(pkg_name, &format!("{}{}", action_name, ACTION_GOAL_SUFFIX), &goal_string)?;
    let result_message = parse_message_string(pkg_name, &format!("{}{}", action_name, ACTION_RESULT_SUFFIX), &result_string)?;
    let feedback_message = parse_message_string(pkg_name, &format!("{}{}", action_name, ACTION_FEEDBACK_SUFFIX), &feedback_string)?;

    Ok(ActionSpecification::new(pkg_name, action_name, goal_message, result_message, feedback_message))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(string: &str) -> anyhow::Result<MessageSpecification> {
        parse_message_string("test_msgs", "Test", string)
    }

    #[test]
    fn parses_single_field_from_single_message() {
        let msg = parse("string name").unwrap();
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(
                Type::new("string", Some("test_msgs")).unwrap(),
                "name",
                None,
            ).unwrap()],
            vec![],
        ).unwrap());
    }

    fn check_parse_string_array_default_value(literal: &str, values: Option<&[&str]>) {
        let msg = parse(&format!("string[] name {}", literal)).unwrap();
        let mut expect = Field::new(
            Type::new("string[]", Some("test_msgs")).unwrap(),
            "name",
            None,
        ).unwrap();
        if let Some(values) = values {
            let mut list = Vec::<PrimitiveType>::new();
            for value in values {
                list.push(PrimitiveType::String(value.to_string()));
            }
            expect.default_value = Some(PrimitiveTypeOrList::List(list));
        }
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![expect],
            vec![],
        ).unwrap());
    }

    #[test]
    fn parses_string_array_default_value_1() {
        check_parse_string_array_default_value("[a,b]", Some(&["a", "b"]));
    }

    #[test]
    fn parses_string_array_default_value_2() {
        check_parse_string_array_default_value("[a,b]#", Some(&["a", "b"]));
    }

    // #[test]
    // NOTE: original rosidl fails this
    // fn parses_string_array_default_value_3() {
    //     check_parse_string_array_default_value("[a,'b#c']", Some(&["a", "b#c"]));
    // }

    #[test]
    fn parses_string_array_default_value_4() {
        check_parse_string_array_default_value("[a,'b]']", Some(&["a", "b]"]));
    }

    #[test]
    fn parses_string_array_default_value_5() {
        check_parse_string_array_default_value("#comment", None);
    }

    #[test]
    fn parses_string_array_default_value_6() {
        check_parse_string_array_default_value(" #comment", None);
    }

    fn check_invalid_string_array_literal(literal: &str) {
        assert!(parse(&format!("string[] name {}", literal)).is_err());

        let msg = parse(&format!("string name {}", literal)).unwrap();
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(
                Type::new("string", Some("test_msgs")).unwrap(),
                "name",
                Some(literal),
            ).unwrap()],
            vec![],
        ).unwrap());

        let msg = parse(&format!("string name {}#comment", literal)).unwrap();
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(
                Type::new("string", Some("test_msgs")).unwrap(),
                "name",
                Some(literal),
            ).unwrap()],
            vec![],
        ).unwrap());

        let msg = parse(&format!("string name {} #comment", literal)).unwrap();
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(
                Type::new("string", Some("test_msgs")).unwrap(),
                "name",
                Some(literal),
            ).unwrap()],
            vec![],
        ).unwrap());
    }

    #[test]
    fn rejects_invalid_string_array_literal_but_accepts_as_string_literal_1() {
        check_invalid_string_array_literal("[,]");
    }

    #[test]
    fn rejects_invalid_string_array_literal_but_accepts_as_string_literal_2() {
        check_invalid_string_array_literal("[,a]");
    }

    #[test]
    fn rejects_invalid_string_array_literal_but_accepts_as_string_literal_3() {
        check_invalid_string_array_literal("[a,']");
    }

    #[test]
    fn rejects_invalid_string_array_literal_but_accepts_as_string_literal_4() {
        check_invalid_string_array_literal("[");
    }

    #[test]
    fn rejects_invalid_string_array_literal_but_accepts_as_string_literal_5() {
        check_invalid_string_array_literal("]");
    }

    #[test]
    fn rejects_invalid_string_array_literal_but_accepts_as_string_literal_6() {
        check_invalid_string_array_literal("[a,b]x");
    }

    #[test]
    fn rejects_valid_token_that_does_not_fully_match_a_parser_rule() {
        assert!(parse("abc").is_err());
    }

    #[test]
    fn rejects_invalid_field_name() {
        for name in &["A", "aB", "a_", "_a", "a__b", "3a"] {
            assert!(parse(&format!("string {}", name)).is_err());
        }
    }

    #[test]
    fn rejects_invalid_constant_name() {
        for name in &["a", "aB", "A_", "_A", "A__B", "3A"] {
            assert!(parse(&format!("string {} = 'x'", name)).is_err());
        }
    }

    #[test]
    fn accepts_valid_field_name() {
        for name in &["a", "foo_bar", "foo1_2bar"] {
            assert_eq!(parse(&format!("string {}", name)).unwrap(), MessageSpecification::new(
                "test_msgs",
                "Test",
                vec![Field::new(
                    Type::new("string", Some("test_msgs")).unwrap(),
                    name,
                    None,
                ).unwrap()],
                vec![],
            ).unwrap());
        }
    }

    #[test]
    fn accepts_valid_constant_name() {
        for name in &["A", "A_B", "FOO1_2BAR"] {
            assert_eq!(parse(&format!("string {} = 'x'", name)).unwrap(), MessageSpecification::new(
                "test_msgs",
                "Test",
                vec![],
                vec![Constant::new("string", name, "x").unwrap()],
            ).unwrap());
        }
    }

    #[test]
    fn ignores_comment_lines() {
        let msg = parse("
# your first name goes here
string first_name

# last name here
### foo bar baz?
string last_name
        ").unwrap();
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![
                Field::new(
                    Type::new("string", Some("test_msgs")).unwrap(),
                    "first_name",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("string", Some("test_msgs")).unwrap(),
                    "last_name",
                    None,
                ).unwrap(),
            ],
            vec![],
        ).unwrap());
    }

    #[test]
    fn parses_variable_length_string_array() {
        let msg = parse("string[] names").unwrap();
        let mut my_type = Type::new("string", Some("test_msgs")).unwrap();
        my_type.is_array = true;
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(my_type, "names", None).unwrap()],
            vec![],
        ).unwrap());
    }

    #[test]
    fn parses_fixed_length_string_array() {
        let msg = parse("string[3] names").unwrap();
        let mut my_type = Type::new("string", Some("test_msgs")).unwrap();
        my_type.is_array = true;
        my_type.array_size = Some(3);
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(my_type, "names", None).unwrap()],
            vec![],
        ).unwrap());
    }

    fn make_constant(type_: &str, name: &str, value: PrimitiveType) -> Constant {
        let mut res = Constant::new(type_, name, &value.to_string()).unwrap();
        res.value = value; // make sure that value is correct
        res
    }

    fn make_field(type_: &str, name: &str, default_value: PrimitiveType) -> Field {
        let mut res = Field::new(Type::new(type_, Some("test_msgs")).unwrap(), name, None).unwrap();
        res.default_value = Some(PrimitiveTypeOrList::Single(default_value));
        res
    }

    #[test]
    fn returns_constants() {
        let msg = parse("
uint32 FOO = 55
int32 BAR=-11 # Comment! # another comment
float32 BAZ= \t -32.25
bool SOME_BOOLEAN = 0
string FOO_STR = 'Foo'    \t
string EMPTY=
#string EXAMPLE=\"#comments\" # are handled properly
string UNQUOTED= Bar
string UNQUOTEDSPACE = Bar Foo
string UNQUOTEDSPECIAL = afse_doi@f4!  :834$%G$%
string BLANK=
string BLANKCOMMENT=# Blank with comment
string BLANKSPACECOMMENT= # Blank with comment after space
#string ESCAPED_QUOTE = \\\\'a#comment
        ").unwrap();

        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![],
            vec![
                make_constant("uint32", "FOO", PrimitiveType::UInt32(55)),
                make_constant("int32", "BAR", PrimitiveType::Int32(-11)),
                make_constant("float32", "BAZ", PrimitiveType::Float32(-32.25)),
                make_constant("bool", "SOME_BOOLEAN", PrimitiveType::Bool(false)),
                make_constant("string", "FOO_STR", PrimitiveType::String("Foo".to_string())),
                make_constant("string", "EMPTY", PrimitiveType::String("".to_string())),
                // make_constant("string", "EXAMPLE", PrimitiveType::String("#comments".to_string())),
                make_constant("string", "UNQUOTED", PrimitiveType::String("Bar".to_string())),
                make_constant("string", "UNQUOTEDSPACE", PrimitiveType::String("Bar Foo".to_string())),
                make_constant("string", "UNQUOTEDSPECIAL", PrimitiveType::String("afse_doi@f4!  :834$%G$%".to_string())),
                make_constant("string", "BLANK", PrimitiveType::String("".to_string())),
                make_constant("string", "BLANKCOMMENT", PrimitiveType::String("".to_string())),
                make_constant("string", "BLANKSPACECOMMENT", PrimitiveType::String("".to_string())),
                // make_constant("string", "ESCAPED_QUOTE", PrimitiveType::String("\\'a".to_string())),
            ],
        ).unwrap());
    }

    #[test]
    fn works_with_python_boolean_values() {
        let msg = parse("
bool ALIVE=True
bool DEAD=False
        ").unwrap();
        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![],
            vec![
                make_constant("bool", "ALIVE", PrimitiveType::Bool(true)),
                make_constant("bool", "DEAD", PrimitiveType::Bool(false)),
            ],
        ).unwrap());
    }

    #[test]
    fn handles_type_names_for_fields() {
        assert_eq!(parse("time time").unwrap(), MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(
                Type::new("time", Some("test_msgs")).unwrap(),
                "time",
                None,
            ).unwrap()],
            vec![],
        ).unwrap());

        assert_eq!(parse("time time_ref").unwrap(), MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![Field::new(
                Type::new("time", Some("test_msgs")).unwrap(),
                "time_ref",
                None,
            ).unwrap()],
            vec![],
        ).unwrap());
    }

    #[test]
    fn parses_unquoted_string_default_or_constant_value() {
        for (literal, value) in &[
            ("x", "x"),
            // ("\\b", "\u{0008}"),
            ("\\p", "\\p"),
            // ("\\foo", "\u{000c}oo"),
            ("[a\\,b]", "[a\\,b]"),
        ] {
            println!("literal: {}, value: {}", literal, value);
            assert_eq!(parse(&format!("string x {}", literal)).unwrap(), MessageSpecification::new(
                "test_msgs",
                "Test",
                vec![Field::new(
                    Type::new("string", Some("test_msgs")).unwrap(),
                    "x",
                    Some(value),
                ).unwrap()],
                vec![],
            ).unwrap());

            assert_eq!(parse(&format!("string X = {}", literal)).unwrap(), MessageSpecification::new(
                "test_msgs",
                "Test",
                vec![],
                vec![make_constant("string", "X", PrimitiveType::String(value.to_string()))],
            ).unwrap());
        }
    }

    #[test]
    fn rejects_literals_of_incorrect_type() {
        assert!(parse("int32 x abc").is_err());
        assert!(parse("bool x abc").is_err());
    }

    #[test]
    fn parses_default_values() {
        let msg = parse(r#"
int8 a 0
int8 b -1
bool c false
bool d False
bool e true
bool f True
string g "hello"
string h 'hello'
string i "'hello'"
string j '"hello"'
string k "\"hello\""
string l '\'hello\''
string m \foo
        "#).unwrap();

        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![
                make_field("int8", "a", PrimitiveType::Int8(0)),
                make_field("int8", "b", PrimitiveType::Int8(-1)),
                make_field("bool", "c", PrimitiveType::Bool(false)),
                make_field("bool", "d", PrimitiveType::Bool(false)),
                make_field("bool", "e", PrimitiveType::Bool(true)),
                make_field("bool", "f", PrimitiveType::Bool(true)),
                make_field("string", "g", PrimitiveType::String("hello".to_string())),
                make_field("string", "h", PrimitiveType::String("hello".to_string())),
                make_field("string", "i", PrimitiveType::String("'hello'".to_string())),
                make_field("string", "j", PrimitiveType::String("\"hello\"".to_string())),
                make_field("string", "k", PrimitiveType::String("\"hello\"".to_string())),
                make_field("string", "l", PrimitiveType::String("'hello'".to_string())),
                make_field("string", "m", PrimitiveType::String("\\foo".to_string())),
            ],
            vec![],
        ).unwrap());
    }

    #[test]
    fn parses_rcl_interfaces_msg_log() {
        let msg = parse_message_string("rcl_interfaces", "Log", r#"
## Severity level constants
##
## These logging levels follow the Python Standard
## https://docs.python.org/3/library/logging.html#logging-levels
## And are implemented in rcutils as well
## https://github.com/ros2/rcutils/blob/35f29850064e0c33a4063cbc947ebbfeada11dba/include/rcutils/logging.h#L164-L172
## This leaves space for other standard logging levels to be inserted in the middle in the future,
## as well as custom user defined levels.
## Since there are several other logging enumeration standard for different implementations,
## other logging implementations may need to provide level mappings to match their internal implementations.
##

# Debug is for pedantic information, which is useful when debugging issues.
byte DEBUG=10

# Info is the standard informational level and is used to report expected
# information.
byte INFO=20

# Warning is for information that may potentially cause issues or possibly unexpected
# behavior.
byte WARN=30

# Error is for information that this node cannot resolve.
byte ERROR=40

# Information about a impending node shutdown.
byte FATAL=50

##
## Fields
##

# Timestamp when this message was generated by the node.
builtin_interfaces/Time stamp

# Corresponding log level, see above definitions.
uint8 level

# The name representing the logger this message came from.
string name

# The full log message.
string msg

# The file the message came from.
string file

# The function the message came from.
string function

# The line in the file the message came from.
uint32 line
        "#).unwrap();

        assert_eq!(msg, MessageSpecification::new(
            "rcl_interfaces",
            "Log",
            vec![
                Field::new(
                    Type::new("builtin_interfaces/Time", Some("rcl_interfaces")).unwrap(),
                    "stamp",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("uint8", Some("rcl_interfaces")).unwrap(),
                    "level",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("string", Some("rcl_interfaces")).unwrap(),
                    "name",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("string", Some("rcl_interfaces")).unwrap(),
                    "msg",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("string", Some("rcl_interfaces")).unwrap(),
                    "file",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("string", Some("rcl_interfaces")).unwrap(),
                    "function",
                    None,
                ).unwrap(),
                Field::new(
                    Type::new("uint32", Some("rcl_interfaces")).unwrap(),
                    "line",
                    None,
                ).unwrap(),
            ],
            vec![
                make_constant("byte", "DEBUG", PrimitiveType::UInt8(10)),
                make_constant("byte", "INFO", PrimitiveType::UInt8(20)),
                make_constant("byte", "WARN", PrimitiveType::UInt8(30)),
                make_constant("byte", "ERROR", PrimitiveType::UInt8(40)),
                make_constant("byte", "FATAL", PrimitiveType::UInt8(50)),
            ],
        ).unwrap());
    }

    #[test]
    fn handles_bound_arrays() {
        // From https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html
        let msg = parse(r#"
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_string_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
        "#).unwrap();

        let mut unbounded_integer_array = Type::new("int32", Some("test_msgs")).unwrap();
        unbounded_integer_array.is_array = true;

        let mut five_integers_array = Type::new("int32", Some("test_msgs")).unwrap();
        five_integers_array.is_array = true;
        five_integers_array.array_size = Some(5);

        let mut up_to_five_integers_array = Type::new("int32", Some("test_msgs")).unwrap();
        up_to_five_integers_array.is_array = true;
        up_to_five_integers_array.array_size = Some(5);
        up_to_five_integers_array.is_upper_bound = true;

        let string_of_unbounded_size = Type::new("string", Some("test_msgs")).unwrap();

        let mut up_to_ten_characters_string = Type::new("string", Some("test_msgs")).unwrap();
        up_to_ten_characters_string.base.string_upper_bound = Some(10);

        let mut up_to_five_unbounded_strings = Type::new("string", Some("test_msgs")).unwrap();
        up_to_five_unbounded_strings.is_array = true;
        up_to_five_unbounded_strings.array_size = Some(5);
        up_to_five_unbounded_strings.is_upper_bound = true;

        let mut unbounded_array_of_string_up_to_ten_characters_each = Type::new("string", Some("test_msgs")).unwrap();
        unbounded_array_of_string_up_to_ten_characters_each.is_array = true;
        unbounded_array_of_string_up_to_ten_characters_each.base.string_upper_bound = Some(10);

        let mut up_to_five_strings_up_to_ten_characters_each = Type::new("string", Some("test_msgs")).unwrap();
        up_to_five_strings_up_to_ten_characters_each.is_array = true;
        up_to_five_strings_up_to_ten_characters_each.is_upper_bound = true;
        up_to_five_strings_up_to_ten_characters_each.array_size = Some(5);
        up_to_five_strings_up_to_ten_characters_each.base.string_upper_bound = Some(10);

        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![
                Field::new(unbounded_integer_array, "unbounded_integer_array", None).unwrap(),
                Field::new(five_integers_array, "five_integers_array", None).unwrap(),
                Field::new(up_to_five_integers_array, "up_to_five_integers_array", None).unwrap(),
                Field::new(string_of_unbounded_size, "string_of_unbounded_size", None).unwrap(),
                Field::new(up_to_ten_characters_string, "up_to_ten_characters_string", None).unwrap(),
                Field::new(up_to_five_unbounded_strings, "up_to_five_unbounded_strings", None).unwrap(),
                Field::new(unbounded_array_of_string_up_to_ten_characters_each, "unbounded_array_of_string_up_to_ten_characters_each", None).unwrap(),
                Field::new(up_to_five_strings_up_to_ten_characters_each, "up_to_five_strings_up_to_ten_characters_each", None).unwrap(),
            ],
            vec![],
        ).unwrap());
    }

    #[test]
    fn handles_bounded_byte_field_with_default_value() {
        // Can be found in /opt/ros/<distro>/share/test_msgs/msg/BoundedSequences.msg
        let msg = parse(r#"
byte[<=3] byte_values_default [0, 1, 255]
        "#).unwrap();

        let mut byte_values_default = make_field("byte", "byte_values_default", PrimitiveType::UInt8(0));
        byte_values_default.default_value = Some(PrimitiveTypeOrList::List(vec![
            PrimitiveType::UInt8(0),
            PrimitiveType::UInt8(1),
            PrimitiveType::UInt8(255),
        ]));
        byte_values_default.type_.is_array = true;
        byte_values_default.type_.array_size = Some(3);
        byte_values_default.type_.is_upper_bound = true;

        assert_eq!(msg, MessageSpecification::new(
            "test_msgs",
            "Test",
            vec![byte_values_default],
            vec![],
        ).unwrap());
    }
}
