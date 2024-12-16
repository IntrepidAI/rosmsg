fn main() {
    let parsed = rosmsg::parse_message_file("rcl_interfaces", "./examples/Log.msg").unwrap();

    println!("Type: {}", parsed.base_type());
    println!("-------------------");
    println!("Constants:");
    for constant in parsed.constants() {
        println!("{} = {}", constant.name(), constant.value());
    }
    println!("-------------------");
    println!("Fields:");
    for field in parsed.fields() {
        println!("{}: {}", field.name(), field.type_());
    }
}
