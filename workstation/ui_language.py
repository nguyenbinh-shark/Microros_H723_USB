"""Vietnamese/English presentation for both desktop apps; never changes robot data.

Keep Vietnamese source messages as catalog keys. Bind only display text, not entry
values, device addresses, commands or map identifiers. Raw device/tool logs stay raw.
"""
import re
import weakref
from functools import lru_cache
from ui_translations import ENGLISH

LANGUAGES = {'vi': 'Tiếng Việt', 'en': 'English'}


def normalize(code):
    return code if isinstance(code, str) and code in LANGUAGES else 'vi'


def _pattern(source):
    parts = re.split(r'(\{\d+\})', source)
    return re.compile(''.join(f'(?P<p{part[1:-1]}>.*?)' if re.fullmatch(r'\{\d+\}', part) else re.escape(part)
                              for part in parts), re.DOTALL)


_TEMPLATES = sorted(((key, value, _pattern(key)) for key, value in ENGLISH.items()
                     if re.search(r'\{\d+\}', key)), key=lambda item: len(re.sub(r'\{\d+\}', '', item[0])), reverse=True)


@lru_cache(maxsize=2048)
def english(text):
    if text in ENGLISH:
        return ENGLISH[text]
    for source, target, pattern in _TEMPLATES:
        match = pattern.fullmatch(text)
        if match:
            values = match.groupdict()
            # A composed message can contain another app status. Unrecognized
            # values (paths, ASCII map names, addresses, numbers) pass through.
            values = {key: english(value) for key, value in values.items()}
            return re.sub(r'\{(\d+)\}', lambda field: values['p' + field[1]], target)
    for separator in ('\n\n', '\n', '  ·  ', ' · ', ' — ', '; ', ': '):
        if separator in text:
            parts = text.split(separator)
            translated = [english(part) for part in parts]
            if translated != parts:
                return separator.join(translated)
    return text


class Dialogs:
    """Translate native dialog labels/messages, keeping paths and options intact."""
    def __init__(self, language, module):
        self.language, self.module = language, module

    def __getattr__(self, name):
        original = getattr(self.module, name)
        def show(*args, **kwargs):
            args = tuple(self.language.text(value) if index < 2 else value for index, value in enumerate(args))
            for key in ('title', 'message', 'prompt', 'detail'):
                if key in kwargs:
                    kwargs[key] = self.language.text(kwargs[key])
            if 'filetypes' in kwargs:
                kwargs['filetypes'] = [(self.language.text(label), pattern) for label, pattern in kwargs['filetypes']]
            return original(*args, **kwargs)
        return show


class Language:
    def __init__(self, code='vi'):
        self.code = normalize(code)
        self.widgets = weakref.WeakKeyDictionary()
        self.variables = {}
        self.titles = weakref.WeakKeyDictionary()
        self.canvases = weakref.WeakKeyDictionary()
        self.trees = weakref.WeakKeyDictionary()
        self.notebooks = weakref.WeakKeyDictionary()

    def text(self, value):
        return english(value) if self.code == 'en' and isinstance(value, str) else value

    def source(self, widget):
        return self.widgets.get(widget, {}).get('text', widget.cget('text'))

    def variable(self, variable):
        """Bind a display StringVar before it reaches Tk's internal label traces."""
        if variable._name in self.variables:
            return
        original = variable.set
        record = {'source': variable.get(), 'set': original}
        self.variables[variable._name] = record
        def changed(value):
            record['source'] = value
            return original(self.text(value))
        variable.set = changed
        original(self.text(record['source']))

    def attach(self, widget):
        """Attach an existing window/subtree once, including future configure calls."""
        import tkinter as tk
        from tkinter import ttk
        if widget not in self.widgets:
            original = widget.configure
            record = {'configure': original}
            self.widgets[widget] = record
            def destroyed(event):
                if event.widget is widget:
                    for records in (self.widgets, self.titles, self.canvases, self.trees, self.notebooks):
                        records.pop(widget, None)
            widget.bind('<Destroy>', destroyed, add='+')
            keys = widget.keys()
            has_variable = 'textvariable' in keys and bool(widget.cget('textvariable'))
            if 'text' in keys and not has_variable:
                record['text'] = widget.cget('text')
                original(text=self.text(record['text']))
            def configure(cnf=None, **kwargs):
                if isinstance(cnf, dict):
                    kwargs = dict(cnf, **kwargs)
                    cnf = None
                if 'text' in kwargs:
                    record['text'] = kwargs['text']
                    kwargs['text'] = self.text(kwargs['text'])
                return original(cnf, **kwargs)
            widget.configure = widget.config = configure
            if isinstance(widget, (tk.Tk, tk.Toplevel)):
                title = widget.title
                title_record = {'source': title(), 'title': title}
                self.titles[widget] = title_record
                def set_title(value=None):
                    if value is None:
                        return title()
                    title_record['source'] = value
                    return title(self.text(value))
                widget.title = widget.wm_title = set_title
                title(self.text(title_record['source']))
            if isinstance(widget, tk.Canvas):
                self._canvas(widget)
            if isinstance(widget, ttk.Treeview):
                self.tree(widget)
            if isinstance(widget, ttk.Notebook):
                tab = widget.tab
                tabs = {name: tab(name, 'text') for name in widget.tabs()}
                self.notebooks[widget] = (tab, tabs)
                def set_tab(tab_id, option=None, **kwargs):
                    if 'text' in kwargs:
                        name = str(widget.tabs()[tab_id]) if isinstance(tab_id, int) else str(tab_id)
                        tabs[name] = kwargs['text']
                        kwargs['text'] = self.text(kwargs['text'])
                    return tab(tab_id, option, **kwargs)
                widget.tab = set_tab
                for name, text in tabs.items():
                    tab(name, text=self.text(text))
        for child in widget.winfo_children():
            self.attach(child)

    def _canvas(self, canvas):
        create, configure, delete = canvas.create_text, canvas.itemconfigure, canvas.delete
        sources = {item: canvas.itemcget(item, 'text') for item in canvas.find_all() if canvas.type(item) == 'text'}
        self.canvases[canvas] = (configure, sources)
        def create_text(*args, **kwargs):
            text = kwargs.get('text', '')
            kwargs['text'] = self.text(text)
            item = create(*args, **kwargs)
            sources[item] = text
            return item
        def itemconfigure(tag, cnf=None, **kwargs):
            if isinstance(cnf, dict):
                kwargs = dict(cnf, **kwargs)
                cnf = None
            if 'text' in kwargs:
                for item in canvas.find_withtag(tag):
                    sources[item] = kwargs['text']
                kwargs['text'] = self.text(kwargs['text'])
            return configure(tag, cnf, **kwargs)
        def remove(*tags):
            for tag in tags:
                for item in canvas.find_withtag(tag):
                    sources.pop(item, None)
            return delete(*tags)
        canvas.create_text, canvas.itemconfigure, canvas.itemconfig, canvas.delete = create_text, itemconfigure, itemconfigure, remove
        for item, text in sources.items():
            configure(item, text=self.text(text))

    def tree(self, tree, *, text=False, columns=()):
        """Opt in to status columns; item IDs, map names and numeric data stay raw."""
        if tree in self.trees:
            self.trees[tree]['text'] = text
            self.trees[tree]['columns'] = columns
            return
        heading, insert, item, delete = tree.heading, tree.insert, tree.item, tree.delete
        record = {'heading': heading, 'item': item, 'headings': {}, 'items': {}, 'text': text, 'columns': columns}
        self.trees[tree] = record
        for column in ('#0', *tree.cget('columns')):
            record['headings'][column] = heading(column, 'text')
            heading(column, text=self.text(record['headings'][column]))
        def options(kwargs):
            display = dict(kwargs)
            if record['text'] and 'text' in display:
                display['text'] = self.text(display['text'])
            if 'values' in display:
                display['values'] = tuple(self.text(value) if index in record['columns'] else value
                                          for index, value in enumerate(display['values']))
            return display
        record['options'] = options
        def set_heading(column, option=None, **kwargs):
            if 'text' in kwargs:
                record['headings'][column] = kwargs['text']
                kwargs['text'] = self.text(kwargs['text'])
            return heading(column, option, **kwargs)
        def add(parent, index, iid=None, **kwargs):
            result = insert(parent, index, iid=iid, **options(kwargs))
            record['items'][result] = dict(kwargs)
            return result
        def set_item(iid, option=None, **kwargs):
            if kwargs:
                record['items'].setdefault(iid, {}).update(kwargs)
            return item(iid, option, **options(kwargs))
        def remove(*items):
            def forget(iid):
                for child in tree.get_children(iid):
                    forget(child)
                record['items'].pop(iid, None)
            for iid in items:
                forget(iid)
            return delete(*items)
        tree.heading, tree.insert, tree.item, tree.delete = set_heading, add, set_item, remove

    def select(self, code):
        self.code = normalize(code)
        for widget, record in list(self.widgets.items()):
            if widget.winfo_exists() and 'text' in record:
                record['configure'](text=self.text(record['text']))
        for record in self.variables.values():
            record['set'](self.text(record['source']))
        for window, record in list(self.titles.items()):
            if window.winfo_exists():
                record['title'](self.text(record['source']))
        for canvas, (configure, sources) in list(self.canvases.items()):
            if canvas.winfo_exists():
                for item, text in sources.items():
                    configure(item, text=self.text(text))
        for tree, record in list(self.trees.items()):
            if tree.winfo_exists():
                for column, text in record['headings'].items():
                    record['heading'](column, text=self.text(text))
                for iid, raw in record['items'].items():
                    if tree.exists(iid):
                        record['item'](iid, **record['options'](raw))
        for notebook, (tab, tabs) in list(self.notebooks.items()):
            if notebook.winfo_exists():
                for name, text in tabs.items():
                    tab(name, text=self.text(text))


def picker(parent, language, on_change, *, background=None):
    import tkinter as tk
    from tkinter import ttk
    frame = tk.Frame(parent, bg=background) if background else ttk.Frame(parent)
    variable = tk.StringVar(frame, value=LANGUAGES[language.code])
    box = ttk.Combobox(frame, textvariable=variable, values=tuple(LANGUAGES.values()), width=11, state='readonly')
    box.pack(fill='x')
    box.bind('<<ComboboxSelected>>', lambda event: on_change(next(code for code, label in LANGUAGES.items() if label == variable.get())))
    frame.variable, frame.combobox = variable, box
    return frame
