### 中文模板 (_drafts/template-bilingual-zh.md)

```markdown
---
title: "您的文章标题"
excerpt: "文章内容的简要描述。"
categories:
  - Category1
  - Category2
tags:
  - tag1
  - tag2
  - tag3
toc: true
toc_label: "目录"
toc_icon: "cog"
lang: zh
font_size: medium  # 可选: small, medium, large
languages:
  - name: "English"
    code: "en"
    url: "/category/subcategory/your-article-slug/"
  - name: "中文"
    code: "zh"
    url: "/category/subcategory/your-article-slug-zh/"
date: YYYY-MM-DD
last_modified_at: YYYY-MM-DD
author_profile: true
---

{% include font-size-control.html %}
{% include language-switcher.html %}

<!-- 简要介绍段落 -->
您的文章介绍放在这里。

<img src="/assets/images/your-image.jpg" alt="您的图片描述" class="embedded-image">

## 主要章节1

第一个主要章节的内容。

### 子章节1.1

更详细的内容在这里：
- 使用项目符号列表
- 在相关时包含代码示例
- 添加图片来说明概念

```language
// 代码示例
您的代码在这里