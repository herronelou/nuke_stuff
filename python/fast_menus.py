"""
Nuke's menu addCommand method is scanning the whole nuke path on every call in search of an icon to use.
This is highly inefficient and results in long start times of nuke as the nuke path grows.

This was reported to Foundry and filed under ID 133546 - Optimize addCommand to save time on Nuke's Launch.

This patch should be removed if Foundry ends up fixing it.
"""

import nuke
# import nuke_internal  # TODO: This doesn't work on nuke 12, need to investigate why, for now skip
import os
import re

ORIGINAL_NUKE_MENU = nuke.menu
# Do the same for nuke_internal ? Used by toolbar
# ORIGINAL_NUKE_INTERNAL_MENU = nuke_internal.menu


def menu_wrapper(method):
    def return_fast_menu(*args, **kwargs):
        """
        Wrap any method so that if they were to return a nuke.Menu, they return a FastMenu instead
        """
        return_value = method(*args, **kwargs)
        if isinstance(return_value, nuke.Menu):
            return FastMenu(return_value)
        return return_value

    return return_fast_menu


class FastMenu(object):

    __class__ = nuke.Menu  # This is required to pass isinstance() checks without inheritance
    icon_cache = {}

    def __init__(self, menu):
        """
        :param menu: Original Nuke Menu
        :type menu: nuke.Menu
        """
        if not isinstance(menu, nuke.Menu):
            raise TypeError('Invalid menu type: {}'.format(type(menu)))
        self._original_menu = menu

    def __getattribute__(self, item):
        try:
            attribute = super(FastMenu, self).__getattribute__(item)
        except AttributeError:
            attribute = self._original_menu.__getattribute__(item)
        if callable(attribute):
            return menu_wrapper(attribute)
        return attribute

    @classmethod
    def _build_icon_cache(cls):
        """
        Create a dictionary of available PNG/SVG files in the nuke path

        :rtype: dict
        """
        cache = {}
        for directory in nuke.pluginPath():
            try:
                files = os.listdir(directory)
            except OSError:
                # It's possible to add non-existent folders to the nuke path
                continue
            for file_name in files:
                name, ext = os.path.splitext(file_name)
                if ext.lower() in ['.png', '.svg']:
                    cache[name] = '{}/{}'.format(directory, file_name)
        cls.icon_cache = cache
        return cache

    def addCommand(self, *args, **kwargs):
        """ Custom implementation of nuke's built-in addCommand, bypassing path crawling """
        name = args[0] if args else kwargs.get('name')
        icon = args[3] if len(args) >= 4 else kwargs.get('icon')
        if not icon:
            icon = name

        # If icon is not a path, look in the cache to find the path nuke would find by itself
        if os.path.basename(icon) == icon:  # No OS separator in string
            icon_name = re.sub(r'\.(png|svg)$', '', icon, flags=re.IGNORECASE)  # Remove ext if present
            cache = FastMenu.icon_cache or FastMenu._build_icon_cache()
            icon = cache.get(icon_name, '/')

        # Modify the icon arg/kwarg
        if len(args) >= 4:
            args = list(args)
            args[3] = icon
        else:
            kwargs['icon'] = icon

        return self._original_menu.addCommand(*args, **kwargs)


def get_fast_menu(*args, **kwargs):
    return menu_wrapper(ORIGINAL_NUKE_MENU)(*args, **kwargs)


def enable_fast_menus():
    """ Monkey patch nuke.menu() to use fast menus """
    nuke.menu = get_fast_menu
    # nuke_internal.menu = get_fast_menu


def disable_fast_menus():
    """ Restore original menus """
    nuke.menu = ORIGINAL_NUKE_MENU
    # nuke_internal.menu = ORIGINAL_NUKE_INTERNAL_MENU
