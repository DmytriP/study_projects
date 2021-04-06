#!/usr/bin/python
# -*- coding: utf-8 -*-
# <Dmytro Poznanskyi><302901>

from typing import Optional, List
from abc import ABC, abstractmethod


class Product:

    def __init__(self, name: str, price: float):
        self.name = name
        self.price = price

    def __hash__(self):
        return hash((self.name, self.price))

    def __eq__(self, other):
        return self.name == other.name and self.price == other.price


class ServerException(Exception):

    def __init__(self, *args):
        super.__init__(*args)


class TooManyProductsFoundError(ServerException):

    def __init__(self, *args):
        super().__init__("Too many products found", *args)


def check_product_name(key: str, n: int) -> bool:
    numbers_in_name = ""
    letters_in_name = ""
    for i in range(len(key)):
        if key[:i].isalpha():
            letters_in_name = key[:i]
            numbers_in_name = key[i:]
    return (len(letters_in_name) is n) and 1 < len(numbers_in_name) < 4


class Servers(ABC):
    n_max_returned_entries = 1000

    def __init__(self, product_list: any):
        self.data = product_list

    @abstractmethod
    def get_entries(self, n_letters) -> List[Product]:
        pass


class ListServer(Servers):

    def __init__(self, product_list: List[Product]):
        super().__init__(product_list)

    def get_entries(self, n_letters: int):
        result = []
        for i in self.data:
            if check_product_name(i.name, n_letters):
                result.append(i)
        return result


class MapServer(Servers):

    def __init__(self, product_list: List[Product]):
        product_dict = {}
        for i in product_list:
            product_dict[i.name] = i
        super().__init__(product_dict)

    def get_entries(self, n_letters: int):
        result = []
        for i in self.data.keys():
            if check_product_name(i, n_letters):
                result.append(self.data[i])
        return result


class Client:

    def __init__(self, server: Servers):
        self.server = server

    def get_total_price(self, n_letters: int):
        try:
            list_prod = self.server.get_entries(n_letters)
        except TooManyProductsFoundError:
            return None
        if not list_prod:
            return None
        price = 0
        for i in list_prod:
            price = price + i.price
        return price
