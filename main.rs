#![no_main]
#![no_std]

use core::mem::size_of;

//======================================
// Standard constants
//======================================
const PAGE_SIZE: usize = 4*1024;

// file descriptors
const STDIN: u32 = 0;
const STDOUT: u32 = 1;
const STDERR: u32 = 2;

// ioctl
const TCGETS: u64 = 0x5401;
const TCSETS: u64 = 0x5402;

// mmap
const MAP_FILE: u32 = 0x00;
const MAP_SHARED: u32 = 0x01;
const MAP_PRIVATE: u32 = 0x02;
const MAP_ANONYMOUS: u32 = 0x20;
const PROT_NONE: u32 = 0x00;
const PROT_READ: u32 = 0x01;
const PROT_WRITE: u32 = 0x02;
const PROT_EXEC: u32 = 0x04;

//======================================
// Minimal runtime
//======================================
#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
#[no_mangle]
#[unsafe(naked)]
extern "C" fn _start() {
    core::arch::naked_asm!(
        "mov rdi, [rsp]",
        "lea rsi, [rsp+8]",
        "call {}",
        "mov rax, 60",
        "xor rdi, rdi",
        "syscall",
        sym main
    );
}

//======================================
// Standard traits
//======================================
trait Write {
    fn write(&mut self, msg: &[u8]);
}
trait Display {
    fn fmt(&self, mut f: Formatter<'_>) {
        f.write(b"???");
    }
}

//======================================
// Standard structs
//======================================
struct File {
    fd: u32,
}

struct Formatter<'a> {
    sink: &'a mut dyn Write,
    depth: usize,
}

struct Vec<const N: usize, T> {
    len: usize,
    arr: [T; N],
}

//======================================
// Standard macros
//======================================
#[macro_export]
macro_rules! const_assert {
    (@size $T:ident == $v:expr) => {
        $crate::const_assert!(@eq ::core::mem::size_of::<$T>(), $v);
    };
    (@align $T:ident == $v:expr) => {
        $crate::const_assert!(@eq ::core::mem::align_of::<$T>(), $v);
    };
    (@offset $T:ty[$field:ident] == $v:expr) => {
        $crate::const_assert!(@eq ::core::mem::offset_of!($T, $field), $v);
    };
    (@eq $actual:expr, $expected:expr) => {
        const _: [(); $expected as usize] = [(); $actual as usize];
    };
    ($cond:expr) => {
        const _: () = if !$cond {
            panic!()
        };
    };
}

#[macro_export]
macro_rules! syscall {
    (read   $($t:tt)*) => { syscall!( 0 => i64 $($t)*) };
    (write  $($t:tt)*) => { syscall!( 1 => i64 $($t)*) };
    (mmap   $($t:tt)*) => { syscall!( 9 => *mut u8 $($t)*) };
    (ioctl  $($t:tt)*) => { syscall!(16 => u64 $($t)*) };
    (fork   $($t:tt)*) => { syscall!(57 => u64 $($t)*) };
    (execve $($t:tt)*) => { syscall!(59 => ! $($t)*) };
    (exit   $($t:tt)*) => { syscall!(60 => ! $($t)*) };
    /*(60 => !, $rdi:expr $(,[$($args:tt)+])?) => {
        core::arch::asm!(
            "mov rax, 60",
            "syscall",
            in("rdi") $rdi,
            options(noreturn)
            $($($args)+)?
        )
    };*/
    ($id:literal => ! $(,$rdi:expr $(,$rsi:expr $(,$rdx:expr $(,$r10:expr $(,$r8:expr $(,$r9:expr)?)?)?)?)?)? $(,[$($args:tt)+])?) => {
        core::arch::asm!(
            concat!("mov rax, ", $id),
            "syscall",
            $(in("rdi") $rdi,
            $(in("rsi") $rsi,
            $(in("rdx") $rdx,
            $(in("r10") $r10,
            $(in("r8")  $r8,
            $(in("r9")  $r9,)?)?)?)?)?)?
            options(noreturn)
            $($($args)+)?
        );
    };
    ($id:literal => $ret:ty $(,$rdi:expr $(,$rsi:expr $(,$rdx:expr $(,$r10:expr $(,$r8:expr $(,$r9:expr)?)?)?)?)?)? $(,[$($args:tt)+])?) => {{
        let ret: $ret;
        core::arch::asm!(
            concat!("mov rax, ", $id),
            "syscall",
            $(in("rdi") $rdi,
            $(in("rsi") $rsi,
            $(in("rdx") $rdx,
            $(in("r10") $r10,
            $(in("r8")  $r8,
            $(in("r9")  $r9,)?)?)?)?)?)?
            lateout("rax") ret,
            clobber_abi("sysv64"),
            $($($args)+)?
        );
        ret
    }};
}

#[macro_export]
macro_rules! print {
    ($($args:expr),* $(,)?) => {
        _printf(&[$(&$args,)*])
    };
}
#[macro_export]
macro_rules! println {
    ($($args:expr),* $(,)?) => {
        print!($($args,)* "\n")
    };
}
#[macro_export]
macro_rules! panic {
    ($($args:expr),* $(,)?) => {
        println!("panic at ", file!(), ":", line!(), "\n", $($args)*);
        unsafe { syscall!(exit, 1) }
    }
}
#[macro_export]
macro_rules! assert {
    ($e:expr) => {
        if !$e {
            panic!("assertion failed");
        }
    }
}
#[macro_export]
macro_rules! impl_display {
    (for $T:ty as slice) => {
        impl Display for $T {
            fn fmt(&self, mut f: Formatter<'_>) {
                f.write(self.as_ref());
            }
        }
    };
    (for $T:ty as unsigned) => {
        impl Display for $T {
            fn fmt(&self, f: Formatter<'_>) {
                _fmt_unsigned(*self as u64, f);
            }
        }
    };
    (for $T:ty as signed) => {
        impl Display for $T {
            fn fmt(&self, f: Formatter<'_>) {
                _fmt_signed(*self as i64, f);
            }
        }
    };
}

//======================================
// Standard implementations
//======================================
impl_display!(for &[u8] as slice);
impl_display!(for &str as slice);
impl_display!(for u8 as unsigned);
impl_display!(for u16 as unsigned);
impl_display!(for u32 as unsigned);
impl_display!(for u64 as unsigned);
impl_display!(for usize as unsigned);
impl_display!(for i8 as signed);
impl_display!(for i16 as signed);
impl_display!(for i32 as signed);
impl_display!(for i64 as signed);
impl_display!(for isize as signed);

fn _fmt_unsigned(mut n: u64, mut f: Formatter<'_>) {
    let mut buf = [0u8; 20];
    for i in 0..20 {
        buf[20-i-1] = (n%10) as u8 + b'0';
        n /= 10;
        if n == 0 {
            f.write(&buf[20-i-1..]);
            break;
        }
    }
}
fn _fmt_signed(n: i64, mut f: Formatter<'_>) {
    let mut buf = [0u8; 20];
    let (s, mut n) = if n < 0 {
        (true, -n)
    } else {
        (false, n)
    };
    for i in 0..19 {
        buf[20-i-1] = (n%10) as u8 + b'0';
        n /= 10;
        if n == 0 {
            if s {
                buf[20-i-2] = b'-';
                f.write(&buf[20-i-2..]);
            } else {
                f.write(&buf[20-i-1..]);
            }
            break;
        }
    }
}

impl Display for char {
    fn fmt(&self, mut f: Formatter<'_>) {
        _panic("char");
    }
}

impl Write for File {
    fn write(&mut self, msg: &[u8]) {
        unsafe { syscall!(write, self.fd, msg.as_ptr(), msg.len()) };
    }
}
impl Write for Formatter<'_> {
    fn write(&mut self, msg: &[u8]) {
        self.sink.write(msg);
    }
}

fn _printf(args: &[&dyn Display]) {
    let mut sink = File { fd: STDOUT };
    for arg in args {
        let f = Formatter {
            sink: &mut sink,
            depth: 0,
        };
        arg.fmt(f);
    }
}

fn _print<T: AsRef<[u8]>>(msg: T) {
    let msg = msg.as_ref();
    unsafe { syscall!(write, STDOUT, msg.as_ptr(), msg.len()) };
}
fn _print0(msg: *const u8) {
    unsafe { syscall!(write, STDOUT, msg, len0(msg)) };
}
fn _panic<T: AsRef<[u8]>>(msg: T) {
    _print(msg);
    unsafe { syscall!(exit, 1) };
}

unsafe fn len0(msg: *const u8) -> usize {
    let mut i = 0;
    while *msg.add(i) != 0 {
        i += 1;
    }
    i
}

impl<const N: usize, T> Vec<N, T> {
    fn new() -> Self {
        Self {
            len: 0,
            arr: unsafe { core::mem::MaybeUninit::uninit().assume_init() }
        }
    }
    fn push(&mut self, e: T) {
        unsafe { *self.arr.get_unchecked_mut(self.len) = e };
        self.len += 1;
    }
    fn insert(&mut self, i: usize, e: T) {
        if i < self.len {
            unsafe { core::ptr::copy(
                self.as_ptr().add(i),
                self.as_mut_ptr().add(i+1),
                self.len - i) };
        }
        unsafe { *self.arr.get_unchecked_mut(i) = e };
        self.len += 1;
    }
    fn remove(&mut self, i: usize) {
        unsafe { core::ptr::copy(
            self.as_ptr().add(i+1),
            self.as_mut_ptr().add(i),
            self.len - i) };
        self.len -= 1;
    }
}

impl<const N: usize, T> core::ops::Deref for Vec<N, T> {
    type Target = [T; N];
    fn deref(&self) -> &Self::Target {
        &self.arr
    }
}
impl<const N: usize, T> core::ops::DerefMut for Vec<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.arr
    }
}

//======================================
// Standard allocator
//======================================
#[inline(never)]
fn malloc(len: usize) -> *mut u8 {
    unsafe { syscall!(mmap, 0, len, 0, 0, -1, 0) }
}

//======================================
// Program
//======================================
#[repr(C)]
#[derive(Clone, Default)]
struct Termios {
    c_iflag: u32,
    c_oflag: u32,
    c_cflag: u32,
    c_lflag: u32,
    c_line: u8,
    c_cc: [u8; 32],
    c_ispeed: u32,
    c_ospeed: u32,
}
// Terminal flags
const ICANON: u32 = 0x00000002;
const ECHO: u32 = 0x00000008;
const ISIG: u32 = 0x00000001;
const IXON: u32 = 0x00000400;
const ICRNL: u32 = 0x00000100;
const OPOST: u32 = 0x00000001;
const VMIN: usize = 6;
const VTIME: usize = 5;

//struct BigLink<const N: usize, T> {
//    next: *const BigLink<N, T>,
//    data: [T; N],
//}
//struct PageLink {
//    next: *const PageLink,
//    data: [u8; PAGE_SIZE - size_of::<*const PageLink>()],
//}
//const_assert!(@size PageLink == PAGE_SIZE);

#[repr(C)]
struct BigLink<const N: usize, Meta> {
    next: *mut BigLink<N, Meta>,
    meta: Meta,
    //_: core::mem::PhantomData<N>,
}

impl<const N: usize, Meta> BigLink<N, Meta> {
    const DATA_SIZE: usize = N*PAGE_SIZE - size_of::<Self>();
    fn new(meta: Meta) -> *mut Self {
        unsafe {
            let ret = syscall!(mmap, 0, N * PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) as *mut Self;
            (*ret).next = 0 as _;
            (*ret).meta = meta;
            ret
        }
    }
    fn as_ref(&self) -> &[u8] {
        unsafe {
            let ptr = (self as *const Self).add(1) as *const u8;
            core::slice::from_raw_parts(ptr, Self::DATA_SIZE)
        }
    }
    fn as_mut(&self) -> &mut[u8] {
        unsafe {
            let ptr = (self as *const Self as *mut Self).add(1) as *mut u8;
            core::slice::from_raw_parts_mut(ptr, Self::DATA_SIZE)
        }
    }
}

impl<const N: usize> BigLink<N, TermMeta> {
    fn push(&mut self, b: u8) {
        if self.meta.len < Self::DATA_SIZE {
            self.as_mut()[self.meta.len] = b;
            self.meta.len += 1;
        } else {
            panic!("Link full");
        }
    }
    fn data(&self) -> &[u8] {
        if self.meta.len < Self::DATA_SIZE {
            &self.as_ref()[..self.meta.len]
        } else {
            panic!("Link full");
        }
    }
}

#[derive(Default)]
struct TermMeta {
    len: usize,
    lines: usize,
    cut: usize,
}

struct Panel {
    x: usize,
    y: usize,
    w: usize,
    h: usize,
    cx: usize,
    cy: usize,
    buffer: *mut BigLink<1, TermMeta>,
}

struct Multiplexer {
    panels: Vec<32, Panel>,
    active: usize,
}

impl Panel {
    fn new(x: usize, y: usize, w: usize, h: usize) -> Self {
        Self {
            x,
            y,
            w,
            h,
            cx: 0,
            cy: 0,
            buffer: BigLink::new(TermMeta::default()),
        }
    }
    fn prompt(&mut self) {
        //const PROMPT: &[u8] = b">>> ";
        const PROMPT: &[u8] = b"";
        unsafe {
            for c in PROMPT {
                (*self.buffer).push(*c);
            }
        }
        self.cx = PROMPT.len();
        print!("\x1b[", self.x+1, "G", PROMPT);
    }
    fn render(&self, index: usize, active: bool) {
        let borders = if active {
            ["━", "┃", "┏", "┓", "┗", "┛"]
            //["═", "║", "╔", "╗", "╚", "╝"]
        } else {
            ["─", "│", "┌", "┐", "└", "┘"]
        };
        print!("\x1b[", self.y, ";", self.x, "H", borders[2]);
        for _ in 0..self.w {
            _print(borders[0]);
        }
        _print(borders[3]);
        print!("\x1b[", self.x+1, "G[", index, "]");
        for y in 0..self.h {
            print!("\x1b[1B\x1b[", self.x, "G", borders[1], "\x1b[", self.w, "C", borders[1]);
        }
        print!("\x1b[1B\x1b[", self.x, "G", borders[4]);
        for _ in 0..self.w {
            _print(borders[0]);
        }
        _print(borders[5]);
    }
}

impl Multiplexer {
    fn new(w: usize, h: usize) -> Self {
        let mut panels = Vec::new();
        panels.push(Panel::new(1, 1, w, h));
        Self {
            panels,
            active: 0,
        }
    }

    fn run(&mut self) {
        let mut skip_render = false;
        let mut prompt = true;
        loop {
            if !skip_render {
                self.render();
            }
            let active = unsafe { self.panels.get_unchecked_mut(self.active) };
            if prompt {
                active.prompt();
            }
            skip_render = false;
            prompt = false;
            let w = active.w;
            let m = unsafe { &mut (*active.buffer).meta };
            let mut x = 0u8;
            unsafe { syscall!(read, STDIN, &mut x, 1) };
            match x {
                3 => break,
                27 => {
                    unsafe { syscall!(read, STDIN, &mut x, 1) };
                    match x {
                        b'[' => {
                            unsafe { syscall!(read, STDIN, &mut x, 1) };
                            match x {
                                b'A' => _print("UP   "),
                                b'B' => _print("DOWN "),
                                b'C' => _print("RIGHT"),
                                b'D' => _print("LEFT "),
                                _ => _panic("")
                            }
                        }
                        b'O' => {
                            unsafe { syscall!(read, STDIN, &mut x, 1) };
                            _print("FN(");
                            _print(&[x]);
                            _print(")");
                        }
                        b'v' => if self.split(true) {
                            prompt = true;
                        }
                        b'h' => if self.split(false) {
                            prompt = true;
                        }
                        b'n' => self.active = if self.active+1 == self.panels.len { 0 } else { self.active+1 },
                        b'p' => self.active = if self.active == 0 { self.panels.len-1 } else { self.active-1 },
                        b'd' => {
                            if self.panels.len == 1 {
                                break;
                            }
                            self.panels.remove(self.active);
                            //for i in self.active..self.panels.len()-1 {
                            //    if i+1 >= self.panels.len {
                            //        break;
                            //    }
                            //    self.panels[i] = self.panels[i+1];
                            //}
                            if self.active == self.panels.len {
                                self.active = self.panels.len-1;
                            }
                            _print("\x1b[2J");
                            prompt = true;
                        },
                        c => {
                            _print("ALT(");
                            _print(&[c]);
                            _print(")");
                        }
                    }
                }
                127 => unsafe {
                    skip_render = true;
                    if (*active.buffer).meta.len == 0 {
                        continue;
                    }
                    (*active.buffer).meta.len -= 1;
                    if active.cx == 0 {
                        active.cx = active.w-1;
                        if active.cy > 0 {
                            active.cy -= 1;
                            _print("\x1b[1A");
                        }
                    } else {
                        active.cx -= 1;
                    }
                    print!("\x1b[", active.x+1+active.cx, "G");
                }
                b'\n' | b'\r' => unsafe {
                    skip_render = true;
                    //if syscall!(fork) == 0 {
                    //    syscall!(execve, (*active.buffer).data().as_ptr(), 0, 0);
                    //}
                    (*active.buffer).push(b'\n');
                    if active.cy < active.h-1 {
                        active.cy += 1;
                        _print("\x1b[1B");
                    }
                    prompt = true;
                }
                c => unsafe {
                    skip_render = true;
                    (*active.buffer).push(c);
                    _print(&[c]);
                    active.cx += 1;
                    if active.cx >= active.w {
                        active.cx = 0;
                        if active.cy < active.h-1 {
                            active.cy +=1;
                            _print("\x1b[1B");
                        }
                        print!("\x1b[", active.x+1, "G");
                    }
                }
            }
        }
    }

    fn split(&mut self, v: bool) -> bool {
        let active = unsafe { self.panels.get_unchecked_mut(self.active) };
        let mut new = if v {
            if active.w < 5*2+2 {
                return false;
            }
            let w = active.w-2;
            active.w = w - w/2;
            Panel::new(active.x + active.w+2, active.y, w/2, active.h)
        } else {
            if active.h < 1*2+2 {
                return false;
            }
            let h = active.h-2;
            active.h = h - h/2;
            Panel::new(active.x, active.y + active.h+2, active.w, h/2)
        };
        //self.active = self.panels.len;
        //self.panels.push(new);
        self.active += 1;
        self.panels.insert(self.active, new);
        true
    }

    fn render(&self) {
        _print("\x1b[?25l");
        for (i, panel) in self.panels.iter().enumerate() {
            if i >= self.panels.len {
                break;
            }
            panel.render(i, i == self.active);
        }
        let active = unsafe { self.panels.get_unchecked(self.active) };
        print!("\x1b[", active.y+active.cy+1, ";", active.x+active.cx+1, "H\x1b[?25h");
    }
}


#[no_mangle]
extern "C" fn main(argc: usize, argv: *const *const u8) {
    //for i in 0..argc {
    //    _print0(unsafe { *argv.add(i) });
    //    _print("\n");
    //}
    let mut termios = Termios {
        c_iflag: 0,
        c_oflag: 0,
        c_cflag: 0,
        c_lflag: 0,
        c_line: 0,
        c_cc: [0; 32],
        c_ispeed: 0,
        c_ospeed: 0,
    };
    let ret = unsafe { syscall!(ioctl, STDIN, TCGETS, &mut termios) };
    if ret != 0 {
        panic!("termios get");
    }
    let mut raw_termios = termios.clone();
    raw_termios.c_lflag &= !(ICANON | ECHO | ISIG);
    raw_termios.c_iflag &= !(IXON | ICRNL);
    raw_termios.c_oflag &= !OPOST;
    raw_termios.c_cc[VMIN] = 1;
    raw_termios.c_cc[VTIME] = 0;

    let ret = unsafe { syscall!(ioctl, STDIN, TCSETS, &raw_termios) };
    if ret != 0 {
        panic!("termios set");
    }

    _print("\x1b[?1049h\x1b[999;999H\x1b[6n");
    let mut x = [0u8; 2];
    let res = unsafe { syscall!(read, STDIN, &mut x, 2) };
    assert!(&x == b"\x1b[");
    let mut rows = 0;
    loop {
        let mut x = 0u8;
        unsafe { syscall!(read, STDIN, &mut x, 1) };
        if x >= b'0' && x <= b'9' {
            rows = rows * 10 + (x - b'0') as usize;
        } else {
            assert!(x == b';');
            break;
        }
    }
    let mut cols = 0;
    loop {
        let mut x = 0u8;
        unsafe { syscall!(read, STDIN, &mut x, 1) };
        if x >= b'0' && x <= b'9' {
            cols = cols * 10 + (x - b'0') as usize;
        } else {
            assert!(x == b'R');
            break;
        }
    }

    let mut m = Multiplexer::new(cols-2, rows-2);
    //m.panels.push(Panel::new(18, 3, 20, 4));
    //m.panels.push(Panel::new(21, 4, 30, 16));
    m.run();
    _print("\x1b[?1049l");

    // reset termios
    unsafe { syscall!(ioctl, STDIN, TCSETS, &termios) };
    println!("rows: ", rows, " cols: ", cols);
}
