extern crate tinyfiledialogs;

use iced_wgpu::{
    canvas,
    container::{Style, StyleSheet},
    Renderer,
};
use iced_winit::{
    button, mouse, Background, Button, Color, Column, Command, Container, Element, Length, Point,
    Program, Radio, Rectangle, Size, Text,
};

use std::cell::Cell;

pub const COLOR_SIZE: f32 = 20.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EditOp {
    Draw,
    Erase,
    Refill,
}

impl EditOp {
    pub const ALL: [EditOp; 3] = [EditOp::Draw, EditOp::Erase, EditOp::Refill];
}

impl Default for EditOp {
    fn default() -> EditOp {
        EditOp::Draw
    }
}

#[derive(Debug, Clone)]
pub enum Message {
    EditChanged(EditOp),
    ExportPressed,
    SavePressed,
    LoadPressed,
    ColorPicked(Color),
}

#[derive(Default)]
struct ColorPicker {
    canvas_cache: canvas::Cache,
    colors: Vec<Color>,
}

impl ColorPicker {
    pub const COLORS_PER_LINE: usize = 6;
    fn new() -> Self {
        let colors = vec![
            // red
            Color::new(1.0, 0.0, 0.0, 1.0),
            // maroon
            Color::new(0.5, 0.0, 0.0, 1.0),
            // orange
            Color::from_rgb8(255, 165, 0),
            // orange red
            Color::from_rgb8(255, 69, 0),
            // yellow
            Color::new(1.0, 1.0, 0.0, 1.0),
            // olive
            Color::new(0.5, 0.5, 0.0, 1.0),
            // brown
            Color::from_rgb8(139, 69, 19),
            // purple
            Color::new(0.5, 0.0, 0.5, 1.0),
            // magenta
            Color::new(1.0, 0.0, 1.0, 1.0),
            // violet
            Color::from_rgb8(148, 0, 211),
            // blue
            Color::new(0.0, 0.0, 1.0, 1.0),
            // navy
            Color::new(0.0, 0.0, 0.5, 1.0),
            // deep sky blue
            Color::from_rgb8(0, 191, 255),
            // aqua marine
            Color::from_rgb8(127, 255, 212),
            // cyan
            Color::new(0.0, 1.0, 1.0, 1.0),
            // lime
            Color::new(0.0, 1.0, 0.0, 1.0),
            // green
            Color::new(0.0, 0.5, 0.0, 1.0),
            // light-green
            Color::from_rgb8(144, 238, 144),
            // forest green
            Color::from_rgb8(34, 139, 34),
            // teal
            Color::new(0.0, 0.5, 0.5, 1.0),
            // chartreuse
            Color::new(0.5, 1.0, 0.0, 1.0),
            // white
            Color::new(1.0, 1.0, 1.0, 1.0),
            // silver
            Color::from_rgb8(192, 192, 192),
            // gray
            Color::new(0.5, 0.5, 0.5, 1.0),
            // black
            Color::new(0.02, 0.02, 0.02, 1.0),
        ];
        ColorPicker {
            canvas_cache: canvas::Cache::default(),
            colors,
        }
    }

    fn draw(&self, frame: &mut canvas::Frame) {
        let box_size = Size {
            width: COLOR_SIZE,
            height: COLOR_SIZE,
        };

        for (i, color) in self.colors.iter().enumerate() {
            let anchor = Point {
                x: (i % Self::COLORS_PER_LINE) as f32 * COLOR_SIZE,
                y: (i / Self::COLORS_PER_LINE) as f32 * COLOR_SIZE,
            };
            frame.fill_rectangle(anchor, box_size, *color);
        }
    }

    pub fn view(&mut self) -> Element<Message, Renderer> {
        let number_of_colors = self.colors.len() as f32;
        let height =
            (number_of_colors / Self::COLORS_PER_LINE as f32).ceil() as u16 * COLOR_SIZE as u16;
        canvas::Canvas::new(self)
            .width(Length::Units(
                Self::COLORS_PER_LINE as u16 * COLOR_SIZE as u16,
            ))
            .height(Length::Units(height))
            .into()
    }
}

#[derive(Default)]
struct PickedColor {
    color: Color,
    canvas_cache: canvas::Cache,
}

impl PickedColor {
    fn new(color: Color) -> Self {
        PickedColor {
            color,
            canvas_cache: canvas::Cache::default(),
        }
    }
    fn draw(&self, frame: &mut canvas::Frame) {
        let box_size_black = Size {
            width: COLOR_SIZE,
            height: COLOR_SIZE,
        };

        let box_size = Size {
            width: COLOR_SIZE - 4.0,
            height: COLOR_SIZE - 4.0,
        };

        frame.fill_rectangle(Point { x: 0.0, y: 0.0 }, box_size_black, Color::BLACK);
        frame.fill_rectangle(Point { x: 2.0, y: 2.0 }, box_size, self.color);
    }

    pub fn view(&mut self) -> Element<Message, Renderer> {
        canvas::Canvas::new(self)
            .width(Length::Units(30))
            .height(Length::Units(30))
            .into()
    }
}

pub struct Controls {
    edit_op: Cell<EditOp>,
    export_button: button::State,
    save_button: button::State,
    load_button: button::State,
    color_picker: ColorPicker,
    picked_color: PickedColor,
    export_file: Cell<Option<String>>,
    save_file: Cell<Option<String>>,
    load_file: Cell<Option<String>>,
    width: u16,
}

impl Controls {
    pub fn new(width: u16) -> Controls {
        Controls {
            edit_op: Cell::new(EditOp::default()),
            export_button: button::State::default(),
            save_button: button::State::default(),
            load_button: button::State::default(),
            color_picker: ColorPicker::new(),
            picked_color: PickedColor::new(Color::new(0.02, 0.02, 0.02, 1.0)),
            export_file: Cell::new(None),
            save_file: Cell::new(None),
            load_file: Cell::new(None),
            width,
        }
    }

    pub fn step_edit_op(&self) {
        match self.edit_op.get() {
            EditOp::Draw => self.edit_op.set(EditOp::Erase),
            EditOp::Erase => self.edit_op.set(EditOp::Refill),
            EditOp::Refill => self.edit_op.set(EditOp::Draw),
        }
    }

    pub fn edit_op(&self) -> EditOp {
        self.edit_op.get()
    }

    pub fn draw_color(&self) -> Color {
        self.picked_color.color
    }

    pub fn export_path(&self) -> Option<String> {
        self.export_file.take()
    }

    pub fn save_path(&self) -> Option<String> {
        self.save_file.take()
    }

    pub fn load_path(&self) -> Option<String> {
        self.load_file.take()
    }
}

impl Program for Controls {
    type Renderer = Renderer;
    type Message = Message;

    fn update(&mut self, message: Message) -> Command<Message> {
        match message {
            Message::EditChanged(op) => self.edit_op.set(op),
            Message::ExportPressed => {
                let file_path = tinyfiledialogs::save_file_dialog("Export to OBJ", "Untitled.obj");

                if file_path.is_some() { self.export_file.set( file_path ); }
            },
            Message::SavePressed => {
                let file_path = tinyfiledialogs::save_file_dialog("Save to RON", "Untitled.ron");

                if file_path.is_some() { self.save_file.set( file_path ); }
            },
            Message::LoadPressed => {
                let file_path = tinyfiledialogs::open_file_dialog("Open RON", "Untitled.ron", None);

                if file_path.is_some() { self.load_file.set( file_path ); }
            },
            Message::ColorPicked(color) => self.picked_color = PickedColor::new(color),
        };

        Command::none()
    }

    fn view(&mut self) -> Element<Message, Renderer> {
        let edit_bar = EditOp::ALL
            .iter()
            .fold(
                Column::new()
                    .width(Length::Units(self.width))
                    .spacing(10)
                    .push(Text::new("Edit state (Press space to step):")),
                |column, state| {
                    column.push(Radio::new(
                        *state,
                        &format!("{:?}", state),
                        Some(self.edit_op.get()),
                        Message::EditChanged,
                    ))
                },
            )
            .push(Text::new("Pick a color"))
            .push(self.color_picker.view())
            .push(Text::new("Draw color"))
            .push(self.picked_color.view())
            .push(
                Button::new(&mut self.export_button, Text::new("Export as .obj"))
                    .on_press(Message::ExportPressed),
            )
            .push(
                Button::new(&mut self.save_button, Text::new("Save to .ron"))
                    .on_press(Message::SavePressed),
            )
            .push(
                Button::new(&mut self.load_button, Text::new("Load from .ron"))
                    .on_press(Message::LoadPressed),
            );

        Container::new(edit_bar)
            .width(Length::Units(self.width))
            .height(Length::Fill)
            .padding(10)
            .style(UiStyle {})
            .into()
    }
}

impl canvas::Program<Message> for ColorPicker {
    fn draw(&self, bounds: Rectangle, _cursor: canvas::Cursor) -> Vec<canvas::Geometry> {
        let theme = self.canvas_cache.draw(bounds.size(), |frame| {
            self.draw(frame);
        });

        vec![theme]
    }

    fn update(
        &mut self,
        event: canvas::Event,
        bounds: Rectangle,
        cursor: canvas::Cursor,
    ) -> Option<Message> {
        match event {
            canvas::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                if let Some(pos) = cursor.position() {
                    let (x_dist, y_dist) = (pos.x - bounds.x, pos.y - bounds.y);
                    if x_dist.is_sign_positive()
                        && y_dist.is_sign_positive()
                        && x_dist < bounds.width
                        && y_dist < bounds.height
                    {
                        let x_pos = x_dist as usize / COLOR_SIZE as usize;
                        let y_pos = y_dist as usize / COLOR_SIZE as usize;
                        let idx = y_pos * Self::COLORS_PER_LINE + x_pos;
                        if idx < self.colors.len() {
                            return Some(Message::ColorPicked(self.colors[idx]));
                        }
                    }
                }
            }
            _other => {}
        }
        None
    }
}

impl canvas::Program<Message> for PickedColor {
    fn draw(&self, bounds: Rectangle, _cursor: canvas::Cursor) -> Vec<canvas::Geometry> {
        let theme = self.canvas_cache.draw(bounds.size(), |frame| {
            self.draw(frame);
        });

        vec![theme]
    }
}

struct UiStyle {}
impl StyleSheet for UiStyle {
    fn style(&self) -> Style {
        Style {
            background: Some(Background::Color(Color::WHITE)),
            border_width: 1,
            border_color: Color::BLACK,
            ..Default::default()
        }
    }
}
