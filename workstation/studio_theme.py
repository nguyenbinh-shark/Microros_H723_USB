"""Paper, ink and blue: shared native Tk styles for the robot studio."""
from tkinter import ttk

PAPER = '#f2f1ee'
PANEL = '#faf9f7'
SUNKEN = '#e9e8e4'
INK = '#101014'
MUTED = '#4b5563'
LINE = '#d4d3cf'
BLUE = '#2d81ff'
BLUE_DARK = '#1659c4'
DANGER = '#b3261e'
MONO = ('Consolas', 9)


def apply(root):
    root.configure(background=PAPER)
    root.option_add('*tearOff', False)
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', background=PAPER, foreground=INK, font=('Segoe UI', 10), bordercolor=LINE)
    style.configure('TFrame', background=PAPER)
    style.configure('Panel.TFrame', background=PANEL)
    style.configure('Side.TFrame', background=SUNKEN)
    style.configure('TLabel', background=PAPER, foreground=INK)
    style.configure('Muted.TLabel', foreground=MUTED)
    style.configure('Mono.TLabel', font=MONO, foreground=MUTED)
    style.configure('Title.TLabel', font=('Segoe UI', 17, 'bold'))
    style.configure('Panel.TLabel', background=PANEL)
    style.configure('PanelMono.TLabel', background=PANEL, foreground=MUTED, font=MONO)
    style.configure('TButton', padding=(12, 9), background=PAPER, foreground=INK,
                    borderwidth=1, relief='solid', focusthickness=2, focuscolor=BLUE, font=('Segoe UI', 9))
    style.map('TButton', background=[('disabled', SUNKEN), ('pressed', LINE), ('active', SUNKEN)],
              foreground=[('disabled', '#989895')], bordercolor=[('focus', BLUE), ('!focus', LINE)])
    style.configure('Compact.TButton', padding=(9, 4), font=('Segoe UI', 9))
    style.configure('Stop.TButton', background='#b91c1c', foreground='#ffffff', bordercolor='#b91c1c',
                    font=('Segoe UI', 10, 'bold'), padding=(12, 8))
    style.map('Stop.TButton', background=[('disabled', SUNKEN), ('active', '#991b1b'), ('!disabled', '#b91c1c')],
              foreground=[('disabled', '#989895'), ('!disabled', '#ffffff')])
    for name, normal, hover, text in (
            ('Primary', BLUE, BLUE_DARK, '#ffffff'), ('Ink', INK, '#303035', '#ffffff'),
            ('Danger', PANEL, '#f5e4e1', DANGER)):
        key = name + '.TButton'
        style.configure(key, background=normal, foreground=text, bordercolor=normal,
                        padding=(13, 10), focuscolor=BLUE)
        style.map(key, background=[('disabled', SUNKEN), ('pressed', hover), ('active', hover), ('!disabled', normal)],
                  foreground=[('disabled', '#989895'), ('!disabled', text)],
                  bordercolor=[('focus', BLUE), ('!focus', LINE if name == 'Danger' else normal)])
    style.configure('Link.TButton', background=PAPER, foreground=MUTED, borderwidth=0,
                    padding=(0, 4), anchor='w', focuscolor=BLUE)
    style.map('Link.TButton', background=[('active', PAPER)], foreground=[('active', BLUE)])
    for name, bg, fg in (('Side.TButton', SUNKEN, MUTED), ('ActiveSide.TButton', INK, '#ffffff')):
        style.configure(name, background=bg, foreground=fg, borderwidth=0,
                        padding=(15, 13), anchor='w', font=('Segoe UI', 10), focuscolor=BLUE)
        style.map(name, background=[('active', LINE if bg == SUNKEN else '#303035'), ('!active', bg)],
                  foreground=[('active', INK if bg == SUNKEN else fg), ('!active', fg)], bordercolor=[('focus', BLUE)])
    style.configure('TEntry', fieldbackground=PANEL, foreground=INK, padding=(10, 9),
                    borderwidth=1, insertcolor=INK, lightcolor=LINE, darkcolor=LINE)
    style.map('TEntry', bordercolor=[('focus', BLUE), ('!focus', LINE)])
    style.configure('TSpinbox', fieldbackground=PANEL, foreground=INK, padding=(6, 7), arrowsize=13)
    style.configure('TCheckbutton', background=PAPER, foreground=MUTED, font=('Segoe UI', 9), padding=3)
    style.map('TCheckbutton', background=[('active', PAPER)], indicatorbackground=[('selected', BLUE), ('!selected', PANEL)])
    style.configure('TLabelframe', background=PAPER, bordercolor=LINE, borderwidth=1, relief='solid')
    style.configure('TLabelframe.Label', background=PAPER, foreground=MUTED, font=('Consolas', 9))
    style.configure('TSeparator', background=LINE)
    style.configure('Workspace.TNotebook', background=PAPER, borderwidth=0, tabmargins=0)
    style.layout('Workspace.TNotebook.Tab', [])
    style.configure('Treeview', background=PANEL, fieldbackground=PANEL, foreground=INK,
                    rowheight=34, borderwidth=0, font=('Segoe UI', 9))
    style.map('Treeview', background=[('selected', '#dce9ff')], foreground=[('selected', BLUE_DARK)])
    style.configure('Treeview.Heading', background=SUNKEN, foreground=MUTED,
                    padding=(8, 10), font=('Segoe UI', 9), relief='flat')
    style.map('Treeview.Heading', background=[('active', LINE)])
    style.configure('Vertical.TScrollbar', background=LINE, troughcolor=PAPER,
                    arrowcolor=MUTED, borderwidth=0, arrowsize=8)
    style.layout('Vertical.TScrollbar', [('Vertical.Scrollbar.trough', {'sticky': 'ns', 'children':
                 [('Vertical.Scrollbar.thumb', {'expand': '1', 'sticky': 'nswe'})]})])
    style.configure('TPanedwindow', background=PAPER, sashwidth=16)
    return style
