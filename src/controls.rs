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

pub const COLOR_SIZE: f32 = 30.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EditOp {
    Draw,
    Erase,
}

impl EditOp {
    pub const ALL: [EditOp; 2] = [EditOp::Draw, EditOp::Erase];
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
    ColorPicked(Color),
}

#[derive(Default)]
struct ColorPicker {
    canvas_cache: canvas::Cache,
}

impl ColorPicker {
    pub const COLORS: [Color; 8] = [
        Color::WHITE,
        Color::BLACK,
        Color {
            r: 1.0,
            g: 0.0,
            b: 0.0,
            a: 1.0,
        },
        Color {
            r: 0.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        },
        Color {
            r: 0.0,
            g: 0.0,
            b: 1.0,
            a: 1.0,
        },
        Color {
            r: 1.0,
            g: 0.0,
            b: 1.0,
            a: 1.0,
        },
        Color {
            r: 1.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        },
        Color {
            r: 0.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        },
    ];
    pub const COLORS_PER_LINE: usize = 4;

    fn draw(&self, frame: &mut canvas::Frame) {
        let box_size = Size {
            width: COLOR_SIZE,
            height: COLOR_SIZE,
        };

        for (i, color) in Self::COLORS.iter().enumerate() {
            let anchor = Point {
                x: (i % Self::COLORS_PER_LINE) as f32 * COLOR_SIZE,
                y: (i / Self::COLORS_PER_LINE) as f32 * COLOR_SIZE,
            };
            frame.fill_rectangle(anchor, box_size, *color);
        }
    }

    pub fn view(&mut self) -> Element<Message, Renderer> {
        canvas::Canvas::new(self)
            .width(Length::Units(
                Self::COLORS_PER_LINE as u16 * COLOR_SIZE as u16,
            ))
            .height(Length::Units(
                Self::COLORS.len() as u16 / Self::COLORS_PER_LINE as u16 * COLOR_SIZE as u16,
            ))
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
        let box_size = Size {
            width: COLOR_SIZE,
            height: COLOR_SIZE,
        };

        frame.fill_rectangle(Point { x: 0.0, y: 0.0 }, box_size, self.color);
    }

    pub fn view(&mut self) -> Element<Message, Renderer> {
        canvas::Canvas::new(self)
            .width(Length::Units(30))
            .height(Length::Units(30))
            .into()
    }
}

pub struct Controls {
    edit_op: EditOp,
    export_button: button::State,
    color_picker: ColorPicker,
    picked_color: PickedColor,
    save_file: Cell<Option<String>>,
}

impl Controls {
    pub fn new() -> Controls {
        Controls {
            edit_op: EditOp::default(),
            export_button: button::State::default(),
            color_picker: ColorPicker::default(),
            picked_color: PickedColor::new(Color::BLACK),
            save_file: Cell::new(None),
        }
    }

    pub fn edit_op(&self) -> EditOp {
        self.edit_op
    }

    pub fn draw_color(&self) -> Color {
        self.picked_color.color
    }

    pub fn save_path(&self) -> Option<String> {
        self.save_file.take()
    }
}

impl Program for Controls {
    type Renderer = Renderer;
    type Message = Message;

    fn update(&mut self, message: Message) -> Command<Message> {
        match message {
            Message::EditChanged(op) => self.edit_op = op,
            Message::ExportPressed => {
                let result = nfd::open_save_dialog(Some("obj"), None).unwrap_or_else(|e| {
                    panic!(e);
                });

                match result {
                    nfd::Response::Okay(file_path) => self.save_file.set(Some(file_path)),
                    _ => {}
                }
            }
            Message::ColorPicked(color) => self.picked_color = PickedColor::new(color),
        };

        Command::none()
    }

    fn view(&mut self) -> Element<Message, Renderer> {
        let edit_bar = EditOp::ALL
            .iter()
            .fold(
                Column::new()
                    .width(Length::Units(150))
                    .spacing(10)
                    .push(Text::new("Edit state:")),
                |column, state| {
                    column.push(Radio::new(
                        *state,
                        &format!("{:?}", state),
                        Some(self.edit_op),
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
            );

        Container::new(edit_bar)
            .width(Length::Units(150))
            .height(Length::Units(400))
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
                    if x_dist.is_sign_positive() && y_dist.is_sign_positive() {
                        let x_pos = x_dist as usize / COLOR_SIZE as usize;
                        let y_pos = y_dist as usize / COLOR_SIZE as usize;
                        let idx = y_pos * Self::COLORS_PER_LINE + x_pos;
                        if idx < Self::COLORS.len() {
                            return Some(Message::ColorPicked(Self::COLORS[idx]));
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
