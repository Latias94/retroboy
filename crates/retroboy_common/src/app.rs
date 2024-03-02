use crate::key::Key;

pub trait App {
    fn init(&mut self);
    fn update(&mut self, screen: &mut [u8]);
    fn handle_key_event(&mut self, key: Key, is_down: bool);
    fn should_exit(&self) -> bool;
    fn exit(&mut self);

    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn scale(&self) -> u32;
    fn title(&self) -> String;
}
