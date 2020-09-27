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
            // orange
            Color::new(1.0, 165.0 / 255.0, 0.0, 1.0),
            // orange red
            Color::new(1.0, 69.0 / 255.0, 0.0, 1.0),
            // yellow
            Color::new(1.0, 1.0, 0.0, 1.0),
            // brown
            Color::new(139.0 / 255.0, 69.0 / 255.0, 19.0 / 255.0, 1.0),
            // purple
            Color::new(0.5, 0.0, 0.5, 1.0),
            // magenta
            Color::new(1.0, 0.0, 1.0, 1.0),
            // violet
            Color::new(148.0 / 255.0, 0.0, 211.0 / 255.0, 1.0),
            // blue
            Color::new(0.0, 0.0, 1.0, 1.0),
            // deep sky blue
            Color::new(0.0, 191.0 / 255.0, 1.0, 1.0),
            // aqua marine
            Color::new(127.0 / 252.0, 1.0, 212.0 / 252.0, 1.0),
            // cyan
            Color::new(0.0, 1.0, 1.0, 1.0),
            // green
            Color::new(0.0, 1.0, 0.0, 1.0),
            // chartreuse
            Color::new(0.5, 1.0, 0.0, 1.0),
            // white
            Color::new(1.0, 1.0, 1.0, 1.0),
            // silver
            Color::new(192.0 / 255.0, 192.0 / 255.0, 192.0 / 255.0, 1.0),
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
    color_picker: ColorPicker,
    picked_color: PickedColor,
    save_file: Cell<Option<String>>,
}

impl Controls {
    pub fn new() -> Controls {
        Controls {
            edit_op: Cell::new(EditOp::default()),
            export_button: button::State::default(),
            color_picker: ColorPicker::new(),
            picked_color: PickedColor::new(Color::new(0.02, 0.02, 0.02, 1.0)),
            save_file: Cell::new(None),
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

    pub fn save_path(&self) -> Option<String> {
        self.save_file.take()
    }
}

impl Program for Controls {
    type Renderer = Renderer;
    type Message = Message;

    fn update(&mut self, message: Message) -> Command<Message> {
        match message {
            Message::EditChanged(op) => self.edit_op.set(op),
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
            );

        Container::new(edit_bar)
            .width(Length::Units(150))
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
